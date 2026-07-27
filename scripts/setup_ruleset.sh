#!/bin/sh
#
# setup_ruleset.sh — create or update the `main` branch ruleset with the GitHub CLI.
#
# NOT RUN AUTOMATICALLY, and not run by any agent: it changes repository settings, which is
# the owner's decision. Read it, then run it yourself:
#
#     ./scripts/setup_ruleset.sh                 # show what would be sent
#     ./scripts/setup_ruleset.sh --apply         # actually create/update the ruleset
#
# What it configures, following docs/CI.md:
#
#   - pull request required before merge (no direct pushes to main)
#   - required status checks, strict (branch must be up to date)
#   - linear history
#   - force pushes blocked, branch deletion blocked
#
# Coverage starts informational and is deliberately NOT in the required list; add it once
# there are enough meaningful tests that a percentage means something.
set -eu

REPO="${DRIFTY_REPO:-nickth3man/drift-c}"
APPLY=0
[ "${1:-}" = "--apply" ] && APPLY=1

payload='{
  "name": "main",
  "target": "branch",
  "enforcement": "active",
  "conditions": {
    "ref_name": { "include": ["refs/heads/main"], "exclude": [] }
  },
  "rules": [
    { "type": "deletion" },
    { "type": "non_fast_forward" },
    { "type": "required_linear_history" },
    {
      "type": "pull_request",
      "parameters": {
        "required_approving_review_count": 0,
        "dismiss_stale_reviews_on_push": true,
        "require_code_owner_review": false,
        "require_last_push_approval": false,
        "required_review_thread_resolution": true
      }
    },
    {
      "type": "required_status_checks",
      "parameters": {
        "strict_required_status_checks_policy": true,
        "required_status_checks": [
          { "context": "quality (format, static analysis, workflow lint)" },
          { "context": "linux headless (gcc)" },
          { "context": "linux headless (clang)" },
          { "context": "windows (MSYS2 UCRT64)" },
          { "context": "sanitizers (ASan + UBSan)" },
          { "context": "scenarios vs merge base" }
        ]
      }
    }
  ]
}'

if [ "$APPLY" -eq 0 ]; then
    echo "Repository: $REPO"
    echo "Would send this ruleset (re-run with --apply to do it):"
    echo "$payload"
    echo
    echo "Required checks are matched by job NAME, exactly as ci.yml and"
    echo "physics-regression.yml spell them. Rename a job and you must update this file."
    exit 0
fi

command -v gh >/dev/null 2>&1 || { echo "gh is not installed." >&2; exit 127; }

existing="$(gh api "repos/$REPO/rulesets" --jq '.[] | select(.name=="main") | .id' 2>/dev/null || true)"

if [ -n "$existing" ]; then
    echo "Updating ruleset $existing on $REPO"
    printf '%s' "$payload" | gh api --method PUT "repos/$REPO/rulesets/$existing" --input -
else
    echo "Creating the 'main' ruleset on $REPO"
    printf '%s' "$payload" | gh api --method POST "repos/$REPO/rulesets" --input -
fi

echo "Done. Verify in Settings -> Rules -> Rulesets."
