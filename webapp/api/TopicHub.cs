using Microsoft.AspNetCore.SignalR;

namespace C2.Api;

public class TopicHub : Hub
{
    private readonly TopicState _state;

    public TopicHub(TopicState state)
    {
        _state = state;
    }

    public override async Task OnConnectedAsync()
    {
        // Replay current topic list to the new client so the dropdown is populated
        // without a separate REST round-trip.
        await Clients.Caller.SendAsync("OnTopicList", _state.Topics);
        await base.OnConnectedAsync();
    }
}

// Singleton holding the latest list of configured topics seen from the sim.
public class TopicState
{
    private readonly object _lock = new();
    private IReadOnlyList<TopicSpecDto> _topics = Array.Empty<TopicSpecDto>();

    public IReadOnlyList<TopicSpecDto> Topics
    {
        get { lock (_lock) return _topics; }
    }

    public void SetTopics(IReadOnlyList<TopicSpecDto> topics)
    {
        lock (_lock) _topics = topics;
    }
}
