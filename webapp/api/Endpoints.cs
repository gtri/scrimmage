using System.Net.Http.Json;

namespace C2.Api;

public static class Endpoints
{
    public static void MapMissionEndpoints(this WebApplication app)
    {
        var launcher = app.Configuration["SCRIMMAGE_LAUNCHER_URL"] ?? "http://scrimmage:5050";

        app.MapGet("/api/topics", (TopicState state) => Results.Ok(state.Topics));

        app.MapPost("/api/topics/{network}/{topic}/publish",
            async (string network, string topic, GenericPublishRequest body, TopicTapClient client) =>
            {
                topic = Uri.UnescapeDataString(topic);
                try
                {
                    var (ok, error) = await client.PublishToTopicAsync(network, topic, body.PayloadJson);
                    if (!ok) return Results.BadRequest(new { ok = false, error });
                    return Results.Ok(new { ok = true });
                }
                catch (Grpc.Core.RpcException ex)
                {
                    return Results.Json(
                        new { ok = false, error = $"TopicTap unavailable: {ex.Status.Detail}" },
                        statusCode: 502);
                }
            });

        app.MapPost("/api/commands/target-assignment",
            async (TargetAssignmentRequest body, TopicTapClient client) =>
            {
                if (body.PredatorId <= 0)
                    return Results.BadRequest(new { ok = false, error = "predatorId must be > 0" });
                if (body.TargetId < 0)
                    return Results.BadRequest(new { ok = false, error = "targetId must be >= 0" });

                var payloadJson = System.Text.Json.JsonSerializer.Serialize(new
                {
                    predatorId = body.PredatorId,
                    targetId = body.TargetId,
                });

                try
                {
                    var (ok, error) = await client.PublishToTopicAsync(
                        "GlobalNetwork", "Commands/TargetAssignment", payloadJson);
                    if (!ok) return Results.BadRequest(new { ok = false, error });
                    return Results.Ok(new { ok = true });
                }
                catch (Grpc.Core.RpcException ex)
                {
                    return Results.Json(
                        new { ok = false, error = $"TopicTap unavailable: {ex.Status.Detail}" },
                        statusCode: 502);
                }
            });

        app.MapGet("/api/missions", async (IHttpClientFactory http) =>
        {
            var client = http.CreateClient();
            var resp = await client.GetAsync($"{launcher}/missions");
            var body = await resp.Content.ReadAsStringAsync();
            return Results.Content(body, "application/json", statusCode: (int)resp.StatusCode);
        });

        app.MapPost("/api/missions/start", async (
            StartMissionRequest req,
            IHttpClientFactory http) =>
        {
            var client = http.CreateClient();
            var resp = await client.PostAsJsonAsync($"{launcher}/missions/start", req);
            var body = await resp.Content.ReadAsStringAsync();
            return Results.Content(body, "application/json", statusCode: (int)resp.StatusCode);
        });

        app.MapPost("/api/missions/stop", async (IHttpClientFactory http) =>
        {
            var client = http.CreateClient();
            var resp = await client.PostAsync($"{launcher}/missions/stop", null);
            var body = await resp.Content.ReadAsStringAsync();
            return Results.Content(body, "application/json", statusCode: (int)resp.StatusCode);
        });

        app.MapPost("/api/missions/pause", async (IHttpClientFactory http) =>
        {
            var client = http.CreateClient();
            var resp = await client.PostAsync($"{launcher}/missions/pause", null);
            var body = await resp.Content.ReadAsStringAsync();
            return Results.Content(body, "application/json", statusCode: (int)resp.StatusCode);
        });

        app.MapPost("/api/missions/resume", async (IHttpClientFactory http) =>
        {
            var client = http.CreateClient();
            var resp = await client.PostAsync($"{launcher}/missions/resume", null);
            var body = await resp.Content.ReadAsStringAsync();
            return Results.Content(body, "application/json", statusCode: (int)resp.StatusCode);
        });

        app.MapGet("/api/missions/report", async (IHttpClientFactory http) =>
        {
            var client = http.CreateClient();
            var resp = await client.GetAsync($"{launcher}/missions/report");
            var body = await resp.Content.ReadAsStringAsync();
            return Results.Content(body, "application/json", statusCode: (int)resp.StatusCode);
        });

        app.MapGet("/api/status", async (IHttpClientFactory http) =>
        {
            var client = http.CreateClient();
            var resp = await client.GetAsync($"{launcher}/status");
            var body = await resp.Content.ReadAsStringAsync();
            return Results.Content(body, "application/json", statusCode: (int)resp.StatusCode);
        });
    }
}

public sealed record GenericPublishRequest(string PayloadJson);

public sealed record TargetAssignmentRequest(int PredatorId, int TargetId);
