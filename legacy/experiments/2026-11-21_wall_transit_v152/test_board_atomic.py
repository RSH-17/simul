from concurrent.futures import ThreadPoolExecutor
import json

from board_atomic import put


def test_concurrent_writers_leave_only_complete_json_and_no_shared_temp(tmp_path):
    path = tmp_path/'latest.json'
    put(path, dict(writer=-1, value='initial'))

    def writer(index):
        for generation in range(100):
            put(path, dict(writer=index, generation=generation, value='x'*1000))
            value = json.loads(path.read_text())
            assert 'writer' in value and value['value'] in ('initial', 'x'*1000)

    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(writer, range(8)))
    assert json.loads(path.read_text())['generation'] == 99
    assert not list(tmp_path.glob('*.tmp'))
