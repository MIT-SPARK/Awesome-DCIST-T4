#!/bin/bash
# Clone submodules with SSH if available, otherwise fall back to HTTPS for public repos.
# hydra_multi is private and will be skipped if SSH is not available.
#
# Usage: ./clone-submodules.sh [path-to-awesome_dcist_t4]
#
# Examples:
#   ./clone-submodules.sh                          # run from awesome_dcist_t4 dir
#   ./clone-submodules.sh /path/to/awesome_dcist_t4

set -euo pipefail

REPO_DIR="${1:-.}"
cd "$REPO_DIR"

if [ ! -f .gitmodules ]; then
    echo "ERROR: .gitmodules not found in $(pwd)"
    exit 1
fi

# Private repos that require SSH (will be skipped if SSH unavailable)
PRIVATE_REPOS=("hydra_multi")

# ---- Test SSH connectivity ----
echo "Testing SSH access to github.com..."
ssh_available=false
if ssh -T git@github.com -o StrictHostKeyChecking=accept-new -o ConnectTimeout=5 2>&1 | grep -q "successfully authenticated"; then
    ssh_available=true
    echo "SSH access available."
else
    echo "SSH not available. Will use HTTPS for public repos."
fi

if $ssh_available; then
    # SSH works: clone everything normally
    echo "Cloning all submodules via SSH..."
    git submodule update --init --recursive
    echo "All submodules cloned successfully."
else
    # SSH not available: rewrite URLs to HTTPS for public repos
    echo "Rewriting submodule URLs to HTTPS..."

    current_submodule=""

    # Parse .gitmodules and rewrite SSH URLs to HTTPS
    while IFS= read -r line; do
        # Match submodule name: [submodule "name"]
        if [[ "$line" == *'[submodule "'* ]]; then
            # Extract name between quotes
            current_submodule="${line#*\"}"
            current_submodule="${current_submodule%\"*}"
        fi

        # Match URL line with SSH format: url = git@github.com:org/repo
        if [[ "$line" == *"git@github.com:"* ]]; then
            repo_path="${line#*git@github.com:}"

            # Check if this is a private repo
            is_private=false
            for p in "${PRIVATE_REPOS[@]}"; do
                if [[ "$current_submodule" == "$p" ]]; then
                    is_private=true
                    break
                fi
            done

            if $is_private; then
                echo "  SKIP (private): $current_submodule"
            else
                https_url="https://github.com/${repo_path}"
                # Ensure .git suffix
                [[ "$https_url" != *.git ]] && https_url="${https_url}.git"
                echo "  HTTPS: $current_submodule -> $https_url"
                git config --local "submodule.${current_submodule}.url" "$https_url"
            fi
        fi
    done < .gitmodules

    echo ""
    echo "Cloning public submodules via HTTPS..."
    # Use || true so failure of private repos doesn't stop the script
    git submodule update --init --recursive || true

    # Report which submodules succeeded
    echo ""
    echo "=== Submodule Status ==="
    git submodule status
    echo ""

    # Check if any private repos were skipped
    for p in "${PRIVATE_REPOS[@]}"; do
        submodule_path=$(git config -f .gitmodules "submodule.${p}.path" 2>/dev/null || echo "")
        if [ -n "$submodule_path" ] && [ ! -f "$submodule_path/.git" ] && [ ! -d "$submodule_path/.git" ]; then
            echo "NOTE: Private submodule '$p' was skipped (SSH required)."
            echo "      The rest of the codebase will build without it."
        fi
    done
fi

echo ""
echo "Submodule setup complete."
