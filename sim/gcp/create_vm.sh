#!/usr/bin/env bash
# Create the sim VM (1x L4, 16 vCPU, 64 GB, 300 GB disk, Ubuntu 22.04 + NVIDIA 595), trying zones until one has
# capacity, then record the zone in vm.env. Then: copy setup_vm.sh over and run it (see docs/SIM.md).
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"; source "$HERE/vm.env"
for z in us-west1-c us-west1-b us-west1-a us-central1-a us-central1-b us-central1-c us-east4-a; do
  echo "== trying $z"
  if gcloud compute instances create "$VM" --project "$PROJECT" --account "$ACCOUNT" --zone "$z" \
      --machine-type g2-standard-16 --accelerator=type=nvidia-l4,count=1 --maintenance-policy TERMINATE \
      --image-family ubuntu-accelerator-2204-amd64-with-nvidia-595 --image-project ubuntu-os-accelerator-images \
      --boot-disk-size 300GB --boot-disk-type pd-balanced --labels purpose=dronehacks 2>&1 | tail -3; then
    if gcloud compute instances describe "$VM" --project "$PROJECT" --account "$ACCOUNT" --zone "$z" >/dev/null 2>&1; then
      sed -i.bak "s/^ZONE=[^ ]*/ZONE=$z/" "$HERE/vm.env" && rm -f "$HERE/vm.env.bak"
      echo "created $VM in $z"; exit 0
    fi
  fi
done
echo "no zone had capacity"; exit 1
