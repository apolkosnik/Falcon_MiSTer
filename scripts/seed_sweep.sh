#!/bin/sh
# Re-fit the current synthesis with other fitter seeds until every timing
# check passes, then assemble.  Leaves the passing seed in the project's .qsf.
#   SEEDS="1 2 3" ./scripts/seed_sweep.sh [Falcon|FalconFPU|Falcon040|Falcon060|TT030]
# Run it after a scripts/build.sh of the same project (it reuses that
# synthesis).  Falcon040/Falcon060 fit in HIGH PERFORMANCE EFFORT, as
# build.sh does; their BALANCED mode is put back at the end.
cd "$(dirname "$0")/.."
Q=/opt/intelFPGA_lite/17.0/quartus/bin
P=${1:-Falcon}
case $P in Falcon|FalconFPU|Falcon040|Falcon060|TT030) ;; *) echo "unknown project $P"; exit 1 ;; esac
case $P in Falcon040|Falcon060)
	sed -i 's/^set_global_assignment -name OPTIMIZATION_MODE .*/set_global_assignment -name OPTIMIZATION_MODE "HIGH PERFORMANCE EFFORT"/' $P.qsf
	trap 'sed -i "s/^set_global_assignment -name OPTIMIZATION_MODE .*/set_global_assignment -name OPTIMIZATION_MODE BALANCED/" $P.qsf' EXIT ;;
esac
for s in ${SEEDS:-2 3 4 5 6 7 8 9 10}; do
	sed -i "s/^set_global_assignment -name SEED .*/set_global_assignment -name SEED $s/" $P.qsf
	$Q/quartus_fit $P > output_files/$P.seed_$s.log 2>&1
	grep -q "Fitter Status : Successful" output_files/$P.fit.summary || { echo "seed $s: fit failed"; continue; }
	$Q/quartus_sta $P >> output_files/$P.seed_$s.log 2>&1
	neg=$(grep -c "Slack : -" output_files/$P.sta.summary)
	worst=$(grep "Slack" output_files/$P.sta.summary | awk '{print $3}' | sort -n | head -1)
	echo "seed $s: negative slacks $neg, worst $worst"
	if [ "$neg" -eq 0 ]; then
		$Q/quartus_asm $P >> output_files/$P.seed_$s.log 2>&1 && echo "TIMING MET with seed $s: output_files/$P.rbf" && exit 0
	fi
done
echo "no seed closed timing"; exit 1
