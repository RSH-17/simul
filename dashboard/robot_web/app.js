const $=id=>document.getElementById(id);
const sec=(value)=>value===undefined||value===null?'수신 전':`${Number(value).toFixed(1)}초 전`;
const number=(value,digits=2)=>value===null||value===undefined?'—':Number(value).toFixed(digits);
function rows(id,items){
  const parent=$(id);parent.replaceChildren();
  for(const [name,value,kind] of items){
    const dt=document.createElement('dt'),dd=document.createElement('dd');
    dt.textContent=name;dd.textContent=String(value??'—');if(kind)dd.className=kind;
    parent.append(dt,dd);
  }
}
function image(id,empty,ready,age,limit,path){
  const img=$(id),placeholder=$(empty);
  if(ready && age!==undefined){
    img.src=`${path}?t=${Date.now()}`;placeholder.hidden=true;
    img.onerror=()=>{placeholder.hidden=false;img.removeAttribute('src')};
  }else{placeholder.hidden=false;img.removeAttribute('src')}
  const label=id==='map-image'?$('map-age'):id==='camera-image'?$('camera-age'):null;
  if(label){label.textContent=sec(age);label.classList.toggle('stale',age===undefined||age>limit)}
}
function render(data){
  const ages=data.ages_s||{},status=data.status||{},scan=data.scan||{},cell=data.start_cell||{};
  const live=(ages.mission_status!==undefined&&ages.mission_status<3)||(ages.raw_scan!==undefined&&ages.raw_scan<3);
  $('connection').textContent=live?'ROS 수신 중':'ROS 데이터 대기 / 끊김';
  $('connection').classList.toggle('offline',!live);
  $('mission-state').textContent=status.state||'상태 대기';
  $('mission-state').className=ages.mission_status>3?'bad':'';
  $('mission-detail').textContent=`${status.active_action||'동작 없음'} · 상태 ${sec(ages.mission_status)}`;
  const cellName={start_free:'빈 공간',start_not_free:'출발 셀 점유',start_unknown:'출발 셀 미관측',start_outside_map:'지도 밖',map_or_pose_unavailable:'지도 / TF 대기'};
  $('start-cell').textContent=cellName[cell.reason]||cell.reason||'대기';
  $('start-cell').className=cell.reason==='start_not_free'?'bad':cell.reason==='start_free'?'good':'';
  $('start-cell-detail').textContent=cell.value===undefined?`지도 ${sec(ages.map)} · TF 확인 중`:`지도 값 ${cell.value} · 셀 (${cell.x}, ${cell.y})`;
  const collision=data.collision_monitor||{};
  $('collision').textContent=collision.action||'상태 대기';
  $('collision').className=collision.action==='STOP'?'bad':'';
  $('collision-detail').textContent=`${collision.polygon||'다각형 미수신'} · ${sec(ages.collision_monitor)}`;
  const ids=status.approached_ids||[];
  $('markers').textContent=`${ids.length} / 3`;
  $('marker-detail').textContent=ids.length?`접근 확인 ID ${ids.join(', ')}`:'접근 확인된 ID 없음';
  rows('scan-details',[
    ['원본 LiDAR',sec(ages.raw_scan),ages.raw_scan>3?'bad':''],
    ['필터 LiDAR',sec(ages.filtered_scan),ages.filtered_scan>3?'bad':''],
    ['원본 최소 거리',scan.min_range_m===null?'반사 없음':`${number(scan.min_range_m,3)} m`],
    ['LiDAR 기준 25cm 미만',scan.within_25cm_from_laser??'—'],
    ['몸체 내부 제거',scan.filtered_frame_matched?`${scan.removed_returns}개`:'동일 시각 프레임 대기',scan.removed_returns>0?'warn':'']
  ]);
  const pose=data.robot_pose;
  rows('nav-details',[
    ['현재 동작',status.active_action||'없음'],
    ['시작 준비',status.readiness_reason||'별도 거절 사유 없음',status.readiness_reason?'warn':''],
    ['로봇 위치',pose?`${number(pose[0])}, ${number(pose[1])} m`:'TF 대기'],
    ['전역 / 로컬 계획',`${data.plans?.global||0} / ${data.plans?.local||0}`],
    ['접촉 감지',status.collisions??'—',status.collisions>0?'bad':''],
    ['센서 중단',status.sensor_interruptions??'—']
  ]);
  rows('ages',[
    ['지도',sec(ages.map),ages.map>5?'bad':''],
    ['로봇 TF',sec(data.pose_age_s),data.pose_age_s>3?'bad':''],
    ['카메라',sec(ages.camera),ages.camera>5?'bad':''],
    ['미션 상태',sec(ages.mission_status),ages.mission_status>3?'bad':''],
    ['충돌 감시',sec(ages.collision_monitor),ages.collision_monitor>3?'bad':'']
  ]);
  image('map-image','map-empty',data.map_ready,ages.map,5,'/api/map.png');
  image('camera-image','camera-empty',data.camera_ready,ages.camera,5,'/api/camera.jpg');
  image('scan-image','scan-empty',data.scan_ready,ages.raw_scan,5,'/api/scan.png');
  $('source').textContent=`osp_mission: ${data.policy_prefix||'설치 경로 확인 불가'} · 읽기 전용`;
  $('domain').textContent=`DOMAIN ${data.ros_domain_id} · ${data.namespace||'루트 네임스페이스'}`;
  const warnings=[];
  if(cell.reason==='start_not_free')warnings.push('출발 위치의 지도 셀이 점유 상태입니다. 원본/필터 LiDAR와 지도상의 로봇 위치를 대조하세요.');
  if(collision.action==='STOP')warnings.push('Nav2 충돌 감시가 STOP 상태입니다.');
  if(!live)warnings.push('ROS 데이터가 갱신되지 않습니다. 도메인·네임스페이스·RMW와 로봇 bringup을 확인하세요.');
  if(live&&ages.map!==undefined&&ages.map>5)warnings.push('지도 업데이트가 5초 넘게 없습니다. 표시된 경로와 출발 셀은 오래된 지도 기준입니다.');
  if(live&&data.pose_age_s!==null&&data.pose_age_s>3)warnings.push('map→base_link TF가 3초 넘게 갱신되지 않아 출발 셀 판정을 숨겼습니다.');
  if(ages.filtered_scan===undefined&&ages.raw_scan!==undefined&&ages.raw_scan<3)warnings.push('필터 스캔이 없습니다. v21 필터가 실행 중인지 확인하세요.');
  $('warning').textContent=warnings.join(' ');$('warning').hidden=!warnings.length;
}
async function poll(){
  try{
    const response=await fetch('/api/state',{cache:'no-store'});
    if(!response.ok)throw new Error(`HTTP ${response.status}`);
    render(await response.json());
  }catch(error){
    $('connection').textContent='보드 서버 연결 끊김';$('connection').classList.add('offline');
    $('warning').textContent=`보드 서버 응답 없음: ${error.message}`;$('warning').hidden=false;
  }
}
poll();setInterval(poll,1000);
