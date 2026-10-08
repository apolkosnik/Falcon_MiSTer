#!/bin/sh
# Re-fit the current synthesis with other fitter seeds until every timing
# check passes, then assemble.  Leaves the passing seed in Falcon.qsf.
cd "$(dirname "$0")/.."
Q=/opt/intelFPGA_lite/17.0/quartus/bin
for s in ${SEEDS:-2 3 4 5 6 7 8 9 10}; do
	sed -i "s/^set_global_assignment -name SEED .*/set_global_assignment -name SEED $s/" Falcon.qsf
	$Q/quartus_fit Falcon > output_files/seed_$s.log 2>&1
	grep -q "Fitter Status : Successful" output_files/Falcon.fit.summary || { echo "seed $s: fit failed"; continue; }
	$Q/quartus_sta Falcon >> output_files/seed_$s.log 2>&1
	neg=$(grep -c "Slack : -" output_files/Falcon.sta.summary)
	worst=$(grep "Slack" output_files/Falcon.sta.summary | awk '{print $3}' | sort -n | head -1)
	echo "seed $s: negative slacks $neg, worst $worst"
	if [ "$neg" -eq 0 ]; then
		$Q/quartus_asm Falcon >> output_files/seed_$s.log 2>&1 && echo "TIMING MET with seed $s: output_files/Falcon.rbf" && exit 0
	fi
done
echo "no seed closed timing"; exit 1
