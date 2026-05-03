using C2.Api;
using Microsoft.EntityFrameworkCore;

var builder = WebApplication.CreateBuilder(args);

// Quiet ASP.NET Core's per-request info logging — at ~100 Hz frame rate the gRPC SendFrame
// calls would otherwise produce hundreds of log lines per second and drown out signal.
// Our own C2.Api.* loggers stay at Information.
builder.Logging.AddFilter("Microsoft.AspNetCore", LogLevel.Warning);
builder.Logging.AddFilter("Microsoft.Hosting.Lifetime", LogLevel.Information);

builder.Services.AddGrpc();
builder.Services.AddSignalR();
builder.Services.AddSingleton<TopicState>();
// Register TopicTapClient as a singleton AND surface that same instance as the
// hosted service. AddHostedService<T> alone only registers the type as IHostedService,
// which means endpoints that try to inject TopicTapClient (e.g., the publish endpoints)
// would fail at request time. The two-line pattern keeps a single instance across both.
builder.Services.AddSingleton<TopicTapClient>();
builder.Services.AddHostedService(sp => sp.GetRequiredService<TopicTapClient>());
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
app.MapHub<TopicHub>("/hubs/topics");
app.MapGet("/health", () => "ok");
app.MapMissionEndpoints();

app.Run();
