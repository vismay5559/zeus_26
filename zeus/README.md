# zeus

Metapackage. Depends on every package the Pi runs, so

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to zeus
```

installs and builds the whole stack. See the [workspace README](../README.md).
