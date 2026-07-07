# ---------------------------------------------------------------------------
# Post-hoc language annotation for recorded episodes.
#
# Given a dataset of already-collected rollouts that all perform the SAME task
# (e.g. raw_data/test = an arm packing boxes), this generates a diverse bank of
# natural-language paraphrases of a base instruction with Claude, then freezes
# ONE paraphrase onto each episode. Distinct paraphrases are spread across the
# episodes so the dataset still has language variety even though each episode's
# instruction is fixed.
#
# Output is a non-destructive sidecar file:  <dataset>/language.json
# The trajectory .pkl files are never touched.
#
# Usage:
#   # 1. Review the generated paraphrase bank without writing anything:
#   python -m bc.annotate_language --dataset test \
#       --instruction "please pack the boxes for me" --dry-run
#
#   # 2. Generate + assign one paraphrase per episode and write language.json:
#   python -m bc.annotate_language --dataset test \
#       --instruction "please pack the boxes for me"
#
# Requires:  pip install anthropic   and Claude credentials
#   (ANTHROPIC_API_KEY, or `ant auth login`; see the error message if unset).
# ---------------------------------------------------------------------------

import argparse
import json
import random
import sys
from pathlib import Path

MODEL = "claude-haiku-4-5"  # cheapest model ($1/$5 per 1M tokens); ample for paraphrasing

# How the paraphrase bank is described to the model. We deliberately push for
# variety across register / verb choice / specificity / phrasing so the bank
# isn't just synonym-swaps of a single sentence.
GENERATION_SYSTEM = (
    "You generate diverse natural-language instructions for a robot manipulation "
    "dataset. Every instruction must command the SAME underlying task with the SAME "
    "meaning as the base instruction the user gives you. Vary the surface form "
    "widely across these axes:\n"
    "  - register: polite/formal <-> terse/imperative\n"
    "  - verb choice: use natural synonyms for the action\n"
    "  - specificity: high-level goal <-> slightly more concrete phrasing\n"
    "  - phrasing: imperative command, question, first-person goal statement\n"
    "Keep each instruction something a human would plausibly say to a robot. Do not "
    "change what the robot is being asked to do. Do not invent new objects, "
    "constraints, counts, or steps that aren't implied by the base instruction. "
    "Keep each under ~15 words."
)

PARAPHRASE_SCHEMA = {
    "type": "object",
    "properties": {
        "paraphrases": {
            "type": "array",
            "items": {"type": "string"},
        }
    },
    "required": ["paraphrases"],
    "additionalProperties": False,
}


def repo_root() -> Path:
    # src/bc/bc/annotate_language.py  ->  repo root is 3 parents up.
    return Path(__file__).resolve().parents[3]


def find_episodes(dataset_dir: Path):
    """Return sorted ep_XXXXX.pkl paths in a dataset directory."""
    episodes = [
        f for f in dataset_dir.iterdir()
        if f.name.startswith("ep_") and f.name.endswith(".pkl")
    ]
    return sorted(episodes, key=lambda x: int(x.name.split("_")[1].split(".")[0]))


def generate_paraphrases(instruction: str, count: int):
    """Ask Claude for `count` diverse paraphrases of `instruction`.

    Returns a de-duplicated list that always includes the base instruction.
    """
    try:
        import importlib.util
        if importlib.util.find_spec("anthropic") is None:
            sys.exit(
                "The 'anthropic' package is not installed for this interpreter "
                f"({sys.executable}).\n  pip install anthropic"
            )
        import anthropic
    except ImportError as e:
        # anthropic is present but failed to import — almost always a sys.path
        # contamination (e.g. a sourced ROS 2 workspace putting an old
        # /usr/lib/python3/dist-packages ahead of this interpreter's packages).
        sys.exit(
            f"'anthropic' is installed but failed to import: {e!r}\n"
            f"Interpreter: {sys.executable}\n"
            "This usually means a sourced ROS 2 workspace is shadowing packages. "
            "Run this script in a shell that has NOT sourced ROS, e.g.:\n"
            "  python src/bc/bc/annotate_language.py --dataset test "
            "--instruction '...'"
        )

    try:
        client = anthropic.Anthropic()
    except Exception as e:  # noqa: BLE001 - surface auth/config problems plainly
        sys.exit(f"Could not initialize the Anthropic client: {e}")

    user_msg = (
        f"Base instruction: \"{instruction}\"\n\n"
        f"Generate {count} diverse paraphrases of this instruction following the "
        f"variation axes. Return them in the `paraphrases` array."
    )

    try:
        response = client.messages.create(
            model=MODEL,
            max_tokens=4000,
            system=GENERATION_SYSTEM,
            messages=[{"role": "user", "content": user_msg}],
            output_config={"format": {"type": "json_schema", "schema": PARAPHRASE_SCHEMA}},
        )
    except TypeError:
        # SDK raises TypeError at request-build time when no credentials resolve.
        sys.exit(
            "No Claude credentials found. Set one in this shell, e.g.:\n"
            "  export ANTHROPIC_API_KEY=sk-ant-...\n"
            "then re-run."
        )
    except anthropic.APIError as e:  # noqa: BLE001
        sys.exit(f"Claude request failed: {e}")

    if response.stop_reason == "refusal":
        sys.exit("Claude declined to generate paraphrases for this instruction.")

    text = next((b.text for b in response.content if b.type == "text"), None)
    if text is None:
        sys.exit("Claude returned no text content.")
    paraphrases = json.loads(text)["paraphrases"]

    # Always include the base instruction, de-dupe while preserving order.
    seen = set()
    bank = []
    for p in [instruction, *paraphrases]:
        key = p.strip().lower()
        if p.strip() and key not in seen:
            seen.add(key)
            bank.append(p.strip())
    return bank


def assign_one_per_episode(episodes, bank, seed):
    """Freeze exactly one paraphrase onto each episode.

    Distinct paraphrases are spread across episodes: the bank is shuffled once
    and cycled, so with N episodes and M paraphrases you get max diversity
    (all distinct while N <= M, then it wraps).
    """
    rng = random.Random(seed)
    order = bank[:]
    rng.shuffle(order)
    return {ep.stem: order[i % len(order)] for i, ep in enumerate(episodes)}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", required=True,
                        help="Dataset name under raw_data/ (e.g. 'test'), or a path to the dataset dir.")
    parser.add_argument("--instruction", required=True,
                        help="Base task instruction to paraphrase, e.g. 'please pack the boxes for me'.")
    parser.add_argument("--task", default=None,
                        help="Short task label stored in language.json (default: derived from the instruction).")
    parser.add_argument("--count", type=int, default=30,
                        help="How many paraphrases to request from Claude (default: 30).")
    parser.add_argument("--seed", type=int, default=0,
                        help="RNG seed for the episode->paraphrase assignment (default: 0, reproducible).")
    parser.add_argument("--dry-run", action="store_true",
                        help="Generate and print the paraphrase bank for review; do not write language.json.")
    args = parser.parse_args()

    dataset_dir = Path(args.dataset)
    if not dataset_dir.exists():
        dataset_dir = repo_root() / "raw_data" / args.dataset
    if not dataset_dir.exists():
        sys.exit(f"Dataset directory not found: {dataset_dir}")

    episodes = find_episodes(dataset_dir)
    if not episodes:
        sys.exit(f"No ep_XXXXX.pkl episodes found in {dataset_dir}")

    print(f"Dataset: {dataset_dir}  ({len(episodes)} episodes)")
    print(f"Base instruction: {args.instruction!r}")
    print(f"Requesting {args.count} paraphrases from {MODEL}...")

    bank = generate_paraphrases(args.instruction, args.count)

    print(f"\nParaphrase bank ({len(bank)}):")
    for i, p in enumerate(bank):
        print(f"  [{i:2d}] {p}")

    if args.dry_run:
        print("\n--dry-run: nothing written. Re-run without --dry-run to assign and save.")
        return

    assignments = assign_one_per_episode(episodes, bank, args.seed)

    out = {
        "task": args.task or args.instruction.strip().rstrip(".!"),
        "base_instruction": args.instruction,
        "model": MODEL,
        "seed": args.seed,
        "paraphrase_bank": bank,
        "episodes": assignments,  # ep_XXXXX -> single frozen instruction
    }
    out_path = dataset_dir / "language.json"
    out_path.write_text(json.dumps(out, indent=2))

    print(f"\nAssigned one paraphrase per episode (seed={args.seed}):")
    for ep in episodes:
        print(f"  {ep.stem}: {assignments[ep.stem]}")
    print(f"\nWrote {out_path}")


if __name__ == "__main__":
    main()
