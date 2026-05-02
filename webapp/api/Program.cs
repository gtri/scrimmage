using C2.Api;

var builder = WebApplication.CreateBuilder(args);

builder.Services.AddGrpc();
builder.Services.AddSignalR();
builder.Services.AddCors(o => o.AddDefaultPolicy(p =>
    p.WithOrigins("http://localhost:5173").AllowAnyHeader().AllowAnyMethod().AllowCredentials()));

// Kestrel serves both gRPC (HTTP/2 cleartext) and HTTP/1.1 (REST + SignalR) on one port
builder.WebHost.ConfigureKestrel(opts =>
{
    opts.ListenAnyIP(8080, l => l.Protocols = Microsoft.AspNetCore.Server.Kestrel.Core.HttpProtocols.Http1AndHttp2);
});

var app = builder.Build();

app.UseCors();
app.MapGrpcService<FrameStreamService>();
app.MapHub<FrameHub>("/hubs/frames");
app.MapGet("/health", () => "ok");

app.Run();
