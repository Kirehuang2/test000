import csv, statistics, sys

f = sys.argv[1] if len(sys.argv) > 1 else 'c:/Users/ikeha/e2_studio/workspace/000/matlab/motor_log_20260325_212934.csv'
rows = list(csv.reader(open(f)))
data = rows[1:]
running = [r for r in data if float(r[1]) != 0]
ids = [float(r[12]) for r in running]
iqs = [float(r[2]) for r in running]
rpms = [float(r[1]) for r in running]
iqrefs = [float(r[13]) for r in running]

print(f"Running: {len(running)} samples")
print(f"RPM: avg={statistics.mean(rpms):.1f} std={statistics.stdev(rpms):.1f}")
print(f"Id:  avg={statistics.mean(ids):.4f} std={statistics.stdev(ids):.4f}")
print(f"Iq:  avg={statistics.mean(iqs):.4f} std={statistics.stdev(iqs):.4f}")
print(f"IqRef: avg={statistics.mean(iqrefs):.4f}")
print()
print("First 5 running:")
for r in running[:5]:
    print(f"  t={float(r[0]):.1f} RPM={r[1]} Iq={r[2]} Id={r[12]} IqRef={r[13]}")
print("Around t=15s:")
mid = [r for r in running if 14.5 < float(r[0]) < 15.5]
for r in mid[:5]:
    print(f"  t={float(r[0]):.1f} RPM={r[1]} Iq={r[2]} Id={r[12]} IqRef={r[13]}")
print("Last 5:")
for r in running[-5:]:
    print(f"  t={float(r[0]):.1f} RPM={r[1]} Iq={r[2]} Id={r[12]} IqRef={r[13]}")
