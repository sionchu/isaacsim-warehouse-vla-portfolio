# Publish with GitHub

Git is initialized locally and Git LFS is enabled. Authenticate the installed GitHub CLI when ready:

```powershell
gh auth login
```

Create a private remote first so large files and project history can be reviewed before publication:

```powershell
cd C:\robotics-portfolio
gh repo create isaacsim-ros2-portfolio --private --source . --remote origin --push
```

When the README, license choice, demo media, and secret scan are ready, change the repository visibility deliberately in GitHub. Do not commit NVIDIA caches, datasets, raw recordings, credentials, or generated ROS build directories.

