# Vendor Code Boundary

This repository does not publish Yahboom source code, prompts, PDFs, media,
driver implementations, or credential-bearing files.

Allowed:

- independently authored typed action schema
- independently authored safety policy
- independently authored replay engine
- architecture-level references to ROS2 concepts such as `/cmd_vel`
- evidence labels pointing to local study observations

Not allowed:

- copying vendor function bodies
- copying API key files or model-provider configuration
- copying tutorial media or PDFs
- copying driver code into the public project
- presenting vendor demo behavior as original work

The project contribution is the guardrail and replay boundary, not the vendor
robot software.
