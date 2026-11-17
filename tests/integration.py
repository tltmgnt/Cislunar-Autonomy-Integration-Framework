"""Exercise actual CLI output and fault transitions, without third-party packages."""
import csv
import math
import pathlib
import subprocess
import sys
import tempfile
binary = str(pathlib.Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory() as tmp:
    for scenario in ('nominal', 'outage', 'dropout', 'thermal'):
        output = pathlib.Path(tmp) / (scenario + '.csv')
        subprocess.run([binary, '--steps', '300', '--scenario', scenario, '--output', str(output)], check=True)
        rows = list(csv.DictReader(output.open()))
        assert len(rows) == 300
        expected = {'nominal': 'nominal', 'outage': 'disconnected', 'dropout': 'safe', 'thermal': 'safe'}[scenario]
        assert rows[150]['mode'] == expected, scenario
        assert rows[0]['mode'] == rows[-1]['mode'] == 'nominal'
        for row in rows:
            for key in ('x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'delta_v'):
                assert math.isfinite(float(row[key]))
            assert math.sqrt(sum(float(row[k])**2 for k in ('ax','ay','az'))) <= 1.000001e-4
            if row['mode'] == 'safe':
                assert all(float(row[k]) == 0 for k in ('ax','ay','az'))
    for args in (['--steps','0'], ['--steps','-1'], ['--steps','10junk'], ['--scenario','bad'], ['--output',str(pathlib.Path(tmp)/'missing'/'x.csv')], ['--steps']):
        assert subprocess.run([binary, *args], capture_output=True).returncode != 0
print('CLI integration checks passed')
