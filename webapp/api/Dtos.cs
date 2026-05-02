namespace C2.Api;

public record Vec3(double X, double Y, double Z);
public record Quat(double W, double X, double Y, double Z);

public record EntityDto(
    int Id,
    int TeamId,
    int SubSwarmId,
    string Type,
    bool Active,
    Vec3 Position,
    Vec3 Velocity,
    Quat Orientation
);

public record FrameDto(double Time, IReadOnlyList<EntityDto> Entities);

public record OriginDto(double Lat, double Lon, double Alt);

public record StartMissionRequest(string Name);

public record MissionStartResponse(string Status, int Pid, string Mission, OriginDto Origin);

public record StatusResponse(string Status, string? Mission, double? UptimeS, OriginDto? Origin);
