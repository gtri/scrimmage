using Google.Protobuf.WellKnownTypes;
using Grpc.Core;
using Microsoft.AspNetCore.SignalR;
using ScrimmageProto;

namespace C2.Api;

/// <summary>
/// gRPC server hosted by the API. SCRIMMAGE pushes frames here; we map them to
/// JSON DTOs and broadcast over SignalR. All other RPCs in the contract are
/// implemented as no-ops so scrimmage doesn't see Unimplemented errors.
/// </summary>
public class FrameStreamService : ScrimmageService.ScrimmageServiceBase
{
    private readonly ILogger<FrameStreamService> _log;
    private readonly IHubContext<FrameHub> _hub;

    public FrameStreamService(ILogger<FrameStreamService> log, IHubContext<FrameHub> hub)
    {
        _log = log;
        _hub = hub;
    }

    // SCRIMMAGE invokes SendFrame at gui_update_period (~100 Hz). Throttle the SignalR
    // fan-out to ~30 Hz so the browser isn't drowned in JSON serialization + Cesium
    // entity updates. Static field is fine: one scrimmage process = one connection.
    private static long _lastBroadcastTicks;
    private const long ThrottleTicks = TimeSpan.TicksPerMillisecond * 33; // ~30 Hz cap

    public override async Task<BlankReply> SendFrame(Frame request, ServerCallContext context)
    {
        var nowTicks = DateTime.UtcNow.Ticks;
        var lastTicks = Interlocked.Read(ref _lastBroadcastTicks);
        if (nowTicks - lastTicks < ThrottleTicks)
        {
            return new BlankReply { Success = 1 }; // ACK scrimmage but skip the broadcast
        }
        Interlocked.Exchange(ref _lastBroadcastTicks, nowTicks);

        var dto = MapFrame(request);
        await _hub.Clients.All.SendAsync("OnFrame", dto, context.CancellationToken);
        return new BlankReply { Success = 1 };
    }

    // SCRIMMAGE also calls SendUTMTerrain, SendShapes, SendContactVisual, SendSimInfo,
    // SendGUIMsg, SendWorldPointClicked, and Ready. Implement as no-ops so the service
    // contract is satisfied — frames are the only thing we consume in the MVP.
    public override Task<BlankReply> SendUTMTerrain(UTMTerrain request, ServerCallContext context)
        => Task.FromResult(new BlankReply { Success = 1 });

    public override Task<BlankReply> SendShapes(Shapes request, ServerCallContext context)
        => Task.FromResult(new BlankReply { Success = 1 });

    public override Task<BlankReply> SendContactVisual(ContactVisual request, ServerCallContext context)
        => Task.FromResult(new BlankReply { Success = 1 });

    public override Task<BlankReply> SendSimInfo(SimInfo request, ServerCallContext context)
        => Task.FromResult(new BlankReply { Success = 1 });

    public override Task<BlankReply> SendGUIMsg(GUIMsg request, ServerCallContext context)
        => Task.FromResult(new BlankReply { Success = 1 });

    public override Task<BlankReply> SendWorldPointClicked(WorldPointClicked request, ServerCallContext context)
        => Task.FromResult(new BlankReply { Success = 1 });

    public override Task<BlankReply> Ready(Empty request, ServerCallContext context)
    {
        _log.LogInformation("SCRIMMAGE Ready ping from {peer}", context.Peer);
        return Task.FromResult(new BlankReply { Success = 1 });
    }

    private static FrameDto MapFrame(Frame f)
    {
        var entities = new List<EntityDto>(f.Contact.Count);
        foreach (var c in f.Contact)
        {
            entities.Add(new EntityDto(
                Id: c.Id?.Id ?? 0,
                TeamId: c.Id?.TeamId ?? 0,
                SubSwarmId: c.Id?.SubSwarmId ?? 0,
                Type: c.Type.ToString(),
                Active: c.Active,
                Position: new Vec3(c.State?.Position?.X ?? 0, c.State?.Position?.Y ?? 0, c.State?.Position?.Z ?? 0),
                Velocity: new Vec3(c.State?.LinearVelocity?.X ?? 0, c.State?.LinearVelocity?.Y ?? 0, c.State?.LinearVelocity?.Z ?? 0),
                Orientation: new Quat(c.State?.Orientation?.W ?? 1, c.State?.Orientation?.X ?? 0, c.State?.Orientation?.Y ?? 0, c.State?.Orientation?.Z ?? 0)
            ));
        }
        return new FrameDto(f.Time, entities);
    }
}
