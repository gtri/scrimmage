using Grpc.Core;  // ReadAllAsync extension method
using Grpc.Net.Client;
using Microsoft.AspNetCore.SignalR;
using ScrimmageC2.Generated.TopicTap;

namespace C2.Api;

/// <summary>
/// BackgroundService that connects outbound to the sim-side TopicTap gRPC server,
/// enumerates topics via ListTopics, opens a StreamTopic server-streaming RPC per
/// topic, and fans each incoming message out to all SignalR clients via TopicHub.
/// Retries indefinitely on any connection failure (500 ms backoff).
/// </summary>
public sealed class TopicTapClient : BackgroundService
{
    private readonly IHubContext<TopicHub> _hub;
    private readonly TopicState _state;
    private readonly ILogger<TopicTapClient> _log;
    private readonly string _addr;

    public TopicTapClient(
        IHubContext<TopicHub> hub,
        TopicState state,
        IConfiguration config,
        ILogger<TopicTapClient> log)
    {
        _hub = hub;
        _state = state;
        _log = log;
        _addr = config["TOPICTAP_GRPC_ADDR"] ?? "http://scrimmage:60001";
    }

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                await RunOnce(stoppingToken);
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                return;
            }
            catch (RpcException rpc) when (rpc.StatusCode == StatusCode.Unavailable)
            {
                // Sim is idle (no mission running) — TopicTap port not bound. Expected
                // background condition; log without stack trace at debug level only.
                _log.LogDebug("TopicTap unavailable (sim idle); retrying in 500ms");
                _state.SetTopics(Array.Empty<TopicSpecDto>());
                await SafeBroadcastTopicList();
            }
            catch (Exception ex)
            {
                _log.LogWarning(ex, "TopicTap connection failed; retrying in 500ms");
                _state.SetTopics(Array.Empty<TopicSpecDto>());
                await SafeBroadcastTopicList();
            }

            try { await Task.Delay(500, stoppingToken); }
            catch (OperationCanceledException) { return; }
        }
    }

    private async Task RunOnce(CancellationToken ct)
    {
        using var channel = GrpcChannel.ForAddress(_addr);
        var client = new TopicTapService.TopicTapServiceClient(channel);

        var topicList = await client.ListTopicsAsync(new ListTopicsRequest(), cancellationToken: ct);
        var dtos = topicList.Topics
            .Select(t => new TopicSpecDto(t.Network, t.Topic, t.TypeName))
            .ToList();

        _state.SetTopics(dtos);
        await _hub.Clients.All.SendAsync("OnTopicList", dtos, ct);

        if (dtos.Count == 0)
        {
            _log.LogInformation("TopicTap connected but no topics configured");
            // Idle wait — sim is up but mission has no TopicTap taps.
            await Task.Delay(Timeout.Infinite, ct);
            return;
        }

        var streamTasks = dtos
            .Select(t => StreamOne(client, t, ct))
            .ToArray();
        await Task.WhenAll(streamTasks);
    }

    private async Task StreamOne(
        TopicTapService.TopicTapServiceClient client,
        TopicSpecDto spec,
        CancellationToken ct)
    {
        var req = new StreamTopicRequest { Network = spec.Network, Topic = spec.Topic };
        using var call = client.StreamTopic(req, cancellationToken: ct);
        await foreach (var msg in call.ResponseStream.ReadAllAsync(ct))
        {
            var dto = new TopicMessageDto(
                msg.Network,
                msg.Topic,
                msg.TypeName,
                msg.TSim,
                msg.PayloadJson);
            await _hub.Clients.All.SendAsync("OnTopicMessage", dto, ct);
        }
    }

    private async Task SafeBroadcastTopicList()
    {
        try { await _hub.Clients.All.SendAsync("OnTopicList", _state.Topics); }
        catch { /* hub may not be up yet; benign */ }
    }
}
