#!/usr/bin/env bash
# Laptop-side control of the cloud sim VM. Needs gcloud logged in (gcloud auth login) with the
# project selected. Works in Git Bash (Windows), macOS and Linux. No keys or secrets live here:
# access is your Google login through IAP + OS Login; the VM has no external IP.
#
#   bash cloudsim/vm.sh create    one-time: Cloud NAT (outbound internet) + the VM   (BILLS)
#   bash cloudsim/vm.sh start     start the stopped VM                                (BILLS while up)
#   bash cloudsim/vm.sh stop      stop it (disk kept, ~no compute charge)
#   bash cloudsim/vm.sh status    is it running?
#   bash cloudsim/vm.sh ssh [cmd] shell (or one command) on the VM through IAP
#   bash cloudsim/vm.sh push      copy this checkout's HEAD to ~/Dronehacks on the VM (replaces it)
#   bash cloudsim/vm.sh delete    delete the VM and its disk (rebuild with create + setup.sh)
set -euo pipefail
PROJECT=${PROJECT:-$(gcloud config get-value project 2>/dev/null)}
ZONE=${ZONE:-us-central1-a}
REGION=${ZONE%-*}
VM=${VM:-cloudsim}
G=(gcloud --project "$PROJECT")

case "${1:-}" in
create)
    "${G[@]}" services enable compute.googleapis.com iap.googleapis.com
    "${G[@]}" compute routers describe cloudsim-router --region "$REGION" >/dev/null 2>&1 ||
        "${G[@]}" compute routers create cloudsim-router --network default --region "$REGION"
    "${G[@]}" compute routers nats describe cloudsim-nat --router cloudsim-router --region "$REGION" >/dev/null 2>&1 ||
        "${G[@]}" compute routers nats create cloudsim-nat --router cloudsim-router --region "$REGION" \
            --auto-allocate-nat-external-ips --nat-all-subnet-ip-ranges
    "${G[@]}" compute instances create "$VM" --zone "$ZONE" \
        --machine-type g2-standard-8 --maintenance-policy TERMINATE \
        --image-family ubuntu-accelerator-2404-amd64-with-nvidia-580 --image-project ubuntu-os-accelerator-images \
        --boot-disk-size 200GB --boot-disk-type pd-balanced \
        --no-address --no-service-account --no-scopes \
        --metadata enable-oslogin=TRUE
    ;;
start)  "${G[@]}" compute instances start "$VM" --zone "$ZONE" ;;
stop)   "${G[@]}" compute instances stop "$VM" --zone "$ZONE" ;;
status) "${G[@]}" compute instances list --format="table(name,zone.basename(),machineType.basename(),status)" ;;
ssh)
    shift
    if [ $# -gt 0 ]; then
        "${G[@]}" compute ssh "$VM" --zone "$ZONE" --tunnel-through-iap --command "$*"
    else
        "${G[@]}" compute ssh "$VM" --zone "$ZONE" --tunnel-through-iap
    fi
    ;;
push)
    tmp=$(mktemp -d)
    git archive --format=tar.gz -o "$tmp/repo.tgz" HEAD
    "${G[@]}" compute scp --zone "$ZONE" --tunnel-through-iap "$tmp/repo.tgz" "$VM":repo.tgz
    "${G[@]}" compute ssh "$VM" --zone "$ZONE" --tunnel-through-iap --command \
        'rm -rf ~/Dronehacks && mkdir ~/Dronehacks && tar -xzf ~/repo.tgz -C ~/Dronehacks && rm ~/repo.tgz && echo pushed'
    rm -rf "$tmp"
    ;;
delete) "${G[@]}" compute instances delete "$VM" --zone "$ZONE" ;;
*)      sed -n '2,15p' "$0"; exit 2 ;;
esac
