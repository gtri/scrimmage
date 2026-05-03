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

public record StartMissionRequest(string Name, double? TimeWarp);

public record MissionStartResponse(string Status, int Pid, string Mission, OriginDto Origin, double? TimeWarp);

public record StatusResponse(string Status, string? Mission, double? UptimeS, OriginDto? Origin);

public record TopicSpecDto(string Network, string Topic, string TypeName);

public record TopicMessageDto(
    string Network,
    string Topic,
    string TypeName,
    double TSim,
    string PayloadJson);
