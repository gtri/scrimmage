using System.Net.Http.Json;

namespace C2.Api;

public static class Endpoints
{
    public static void MapMissionEndpoints(this WebApplication app)
    {
        var launcher = app.Configuration["SCRIMMAGE_LAUNCHER_URL"] ?? "http://scrimmage:5050";

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

        app.MapGet("/api/status", async (IHttpClientFactory http) =>
        {
            var client = http.CreateClient();
            var resp = await client.GetAsync($"{launcher}/status");
            var body = await resp.Content.ReadAsStringAsync();
            return Results.Content(body, "application/json", statusCode: (int)resp.StatusCode);
        });
    }
}
