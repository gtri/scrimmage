using Microsoft.EntityFrameworkCore;

namespace C2.Api;

public class AppDbContext : DbContext
{
    public AppDbContext(DbContextOptions<AppDbContext> options) : base(options) { }

    // No entities yet — added in post-MVP iterations (commands, audit log, tags).
}
