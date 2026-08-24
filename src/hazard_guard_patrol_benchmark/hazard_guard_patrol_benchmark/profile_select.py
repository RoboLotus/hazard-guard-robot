from __future__ import annotations

import argparse
import json
from pathlib import Path

from .profile import aggregate_profiles


def main() -> None:
    parser = argparse.ArgumentParser(description="Select stable Docker simulation profiles.")
    parser.add_argument("inputs", nargs="+")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    documents = [
        json.loads(Path(path).read_text(encoding="utf-8")) for path in args.inputs
    ]
    summary = aggregate_profiles(documents)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(output)
    print(json.dumps(summary["recommended"], ensure_ascii=False))


if __name__ == "__main__":
    main()
