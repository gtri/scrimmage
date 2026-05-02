using C2.Api;
using Microsoft.EntityFrameworkCore;

var builder = WebApplication.CreateBuilder(args);

builder.Services.AddGrpc();
builder.Services.AddSignalR();
builder.Services.AddCors(o => o.AddDefaultPolicy(p =>
    p.WithOrigins("http://localhost:5173").AllowAnyHeader().AllowAnyMethod().AllowCredentials()));
builder.Services.AddHttpClient();
builder.Services.AddDbContext<AppDbContext>(opts =>
    opts.UseNpgsql(builder.Configuration.GetConnectionString("Default")));

// Two listeners on separate ports because HTTP/2 cleartext requires either TLS (for ALPN)
// or a port dedicated to HTTP/2 — Kestrel falls back to HTTP/1.1 on a multiplexed cleartext port.
//   :8080  — HTTP/1.1 for REST + SignalR (web client → API)
//   :50051 — HTTP/2 cleartext for gRPC (scrimmage process → API)
builder.WebHost.ConfigureKestrel(opts =>
{
    opts.ListenAnyIP(8080, l => l.Protocols = Microsoft.AspNetCore.Server.Kestrel.Core.HttpProtocols.Http1);
    opts.ListenAnyIP(50051, l => l.Protocols = Microsoft.AspNetCore.Server.Kestrel.Core.HttpProtocols.Http2);
});

var app = builder.Build();

app.UseCors();
app.MapGrpcService<FrameStreamService>();
app.MapHub<FrameHub>("/hubs/frames");
app.MapGet("/health", () => "ok");
app.MapMissionEndpoints();

app.Run();
