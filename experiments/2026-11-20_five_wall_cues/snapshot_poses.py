"""Export map pose at actual decision snapshots for comparison with synchronized Gazebo truth."""
from pathlib import Path
import json,sys
import numpy as np
folder = Path(sys.argv[1])
rows = []
for p in sorted((folder/'native/policy_decisions').glob('*.npz')):
    with np.load(p,allow_pickle=False) as z:
        rows.append([float(z['elapsed']),*map(float,z['robot'][:2]),float(z['heading'])])
print(json.dumps(rows))
