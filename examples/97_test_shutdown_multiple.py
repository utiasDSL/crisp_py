"""Test shutdown with multiple robots."""

import time
from argparse import ArgumentParser
from crisp_py.robot import Robot
from rich import print


parser = ArgumentParser(description="Test shutdown with multiple clients.")
parser.add_argument(
    "-n", type=int, default=10, help="Number of clients to create."
)
parser.add_argument(
    "--namespace", type=str, default="left", help="Namespace for the robots."
)
parser.add_argument(
    "-t", type=int, default=5, help="Time to wait before shutting down (seconds)."
)
args = parser.parse_args()

print(f"Creating {args.n} [b]robots[/b]...")

robots = []
for i in range(args.n):
    robot = Robot(name=f"robot{i}", namespace=f"{args.namespace}")
    robot.wait_until_ready()
    robots.append(robot)

print("[b]Robots created successfully.[/b]")
print(f"Waiting {args.t} seconds for user to check the robots...")
time.sleep(args.t)

# %%

print("[b]Starting shutdown process...[/b]")
for i, robot in enumerate(robots):
    print(f"[red]Shuting down robot {i}.[/red]")
    robots[i].shutdown()
    print(f"[green]Succeded with robot {i}.[/green]")

print("[blue]All robots have been shut down successfully.[/blue]")

