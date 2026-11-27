"""Run the passive strict judge for one team trial with the relocatable launch."""
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from judge.team import PassiveExamTrial, WallTrial, main  # noqa: E402

if __name__ == '__main__':
    random_exam = os.environ.get('TEAM_RANDOM_EXAM') == '1'
    main(PassiveExamTrial if random_exam else WallTrial, HERE/'portable.launch.py')
