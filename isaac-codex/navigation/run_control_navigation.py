"""Desktop launcher using the same Isaac VS Code connection as check_isaac.py."""
from pathlib import Path
from isaac_mcp_server import check_isaac_connection, execute_isaac_code


if __name__ == '__main__':
    status=check_isaac_connection()
    print(status)
    if 'NOT reachable' in status:
        raise SystemExit(1)
    path=str(Path(__file__).with_name('start_control_navigation.py').resolve())
    # repr safely embeds the filesystem path as a Python string, not shell code.
    code=f'exec(compile(open({path!r}).read(), {path!r}, "exec"))'
    print(execute_isaac_code(code))
    print('Startup scheduled in Isaac. Check the Warehouse Control & Navigation panel.')
