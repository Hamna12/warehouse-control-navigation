"""Execute this file inside Isaac Sim's Script Editor or VS Code executor."""
import asyncio
from pathlib import Path
import sys

navigation_dir=Path('/home/hmna/Downloads/isaac-codex/navigation')
if str(navigation_dir) not in sys.path:
    sys.path.insert(0,str(navigation_dir))
from control_nav.isaac_app import launch
warehouse_navigation_start_task=asyncio.ensure_future(launch())
