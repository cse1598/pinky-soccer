# Legacy source baseline

Use a separate clean checkout of the recorded HEAD and pinned submodules.
Overlay source.tar.gz there; never extract over the active workspace.
The archive includes uncommitted source/config changes. Verify the archive with
`python scripts/capture_control_baseline.py --check PATH`.
Binary assets, calibration sessions and native build products are not bundled.
Rebuild in the recorded environment and verify the selected runtime profile hashes.
Git patches are supplementary; the archive is the dirty source reference.
