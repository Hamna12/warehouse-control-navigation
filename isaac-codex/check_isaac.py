"""Check Isaac Sim and inspect its stage without opening or modifying a stage."""

import sys

from isaac_mcp_server import check_isaac_connection, execute_isaac_code


def main():
    status = check_isaac_connection()
    print(status, flush=True)
    if "NOT reachable" in status:
        return 1
    result = execute_isaac_code(
        "import omni.usd\n"
        "from pathlib import Path\n"
        "stage = omni.usd.get_context().get_stage()\n"
        "print('Loaded stage:', stage.GetRootLayer().identifier if stage else None)\n"
        "path = Path('/home/hmna/Downloads/Team_Warehouse_MobilityGen.usd')\n"
        "with path.open('rb') as stream:\n"
        "    print('Warehouse readable; header:', repr(stream.read(16)))\n"
    )
    print(result)
    return 0 if "Warehouse readable; header:" in result else 1


if __name__ == "__main__":
    sys.exit(main())
