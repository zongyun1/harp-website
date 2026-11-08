import * as T from 'three';
import {GLTFLoader} from './vendor/loaders/GLTFLoader.js';
import {GroundedSkybox} from './vendor/objects/GroundedSkybox.js';
import RAPIER from './vendor/rapier.mjs';
import {Robot, capsuleContact} from './robot.js';
const $=id=>document.getElementById(id), keys=new Set();
const V=a=>new T.Vector3().fromArray(a), Q=a=>new T.Quaternion(a[1],a[2],a[3],a[0]);
const DT=1/60;
const embedded=document.documentElement.classList.contains('embedded');
let embeddedVisible=true;
let status='loading', elapsed=0, motionTime=null, human='Waiting', contact=null;
let world, robot, object, objectView, objectInfo, handBody, gripJoint=null, closed=false, opening=1;
let meta, scene, camera, renderer, avatar, skinFrames, idleSkin, capFrames, capsules;
let linkViews=[], physicsLinks=[], collisionPoints=[], heldByHuman=false, humanCanHold=true, frameCount=0, lastMetric=performance.now(), frameRate=0;
const errorMessage=error=>{console.error(error);status='error';$('detail').textContent=error.message;$('overlay-title').textContent='Unable to start the browser demo';$('overlay-detail').textContent=error.message;$('overlay').hidden=false;$('status').textContent='Loading failed';};
async function getJSON(path){const r=await fetch(path);if(!r.ok)throw Error(`Could not load ${path} (${r.status})`);return r.json();}
async function getBinary(desc,Type=Float32Array){const r=await fetch(`assets/${desc.file}?v=aligned-start-v3`);if(!r.ok)throw Error(`Could not load ${desc.file}`);return new Type(await r.arrayBuffer());}
function makeApple(){
 // Match the exported fruit collider (roughly 5 cm across) so the visual,
 // recorded human hand pose, and robot grasp continue to agree.
 const fruit=new T.Group();
 const profile=[[0,.003],[.012,.002],[.023,.006],[.029,.017],[.029,.030],[.024,.040],[.015,.047],[.006,.045],[0,.043]];
 const body=new T.Mesh(
  new T.LatheGeometry(profile.map(([r,y])=>new T.Vector2(r,y)),48),
  new T.MeshBasicMaterial({color:0xe43729,side:T.DoubleSide})
 );
 body.castShadow=true;body.receiveShadow=true;fruit.add(body);
 const stem=new T.Mesh(new T.CylinderGeometry(.0013,.0018,.014,8),new T.MeshStandardMaterial({color:0x574027,roughness:.9}));
 stem.position.set(0,.051,0);stem.rotation.z=-.18;stem.castShadow=true;fruit.add(stem);
 const leaf=new T.Mesh(new T.SphereGeometry(.008,12,8),new T.MeshStandardMaterial({color:0x4c9a4d,roughness:.7,side:T.DoubleSide}));
 leaf.scale.set(1,.18,.5);leaf.position.set(.007,.052,.001);leaf.rotation.z=.35;leaf.castShadow=true;fruit.add(leaf);
 return fruit;
}
function clearKeys(){keys.clear();document.querySelectorAll('[data-key]').forEach(b=>b.classList.remove('active'));}
function updateUI(){
 const titles={loading:'Loading…',ready:'Ready to play',running:'Control active',paused:'Paused',failed:'Collision detected — failure',success:'Task completed — success'};
 $('status').textContent=titles[status]||status;$('status').className=`status ${status}`;
 $('overlay').hidden=status==='running';$('overlay-title').textContent=titles[status]||status;
 const detail=status==='failed'?`Robot–human contact (${contact.link.replaceAll('_',' ')}). Reset to try again.`:status==='success'?'Apple in the bin. The human inspection is complete.':status==='ready'?'Press Start task, then use W/A/S/D and Q/E to move the robot.':status==='paused'?'Press Resume or Space to continue.':'Let the human finish inspecting, then pick up the apple and drop it in the bin.';
 $('detail').textContent=detail;$('overlay-detail').textContent=detail;
 $('human').textContent=human;$('time').textContent=`${elapsed.toFixed(1)} s`;
 $('phase').textContent=human==='Finished'?'Human finished · place the apple in the bin':human==='Waiting'?'Human will inspect the apple shortly':`Human ${human.toLowerCase()} · keep clear`;
 $('grip-state').textContent=gripJoint?'Holding apple':closed?'Closed':'Open';
 $('gripper').textContent=closed?'G · open gripper':'G · close gripper';
 $('start').disabled=!['ready','paused'].includes(status);$('start').textContent=status==='paused'?'Resume':'Start task';
 $('pause').disabled=status!=='running';$('reset').disabled=status==='loading';$('gripper').disabled=status!=='running';
 document.querySelectorAll('[data-key]').forEach(b=>b.disabled=status!=='running');
}
function pause(){if(status==='running'){status='paused';clearKeys();updateUI();}}
function start(){if(!['ready','paused'].includes(status))return;clearKeys();status='running';$('stage').focus({preventScroll:true});updateUI();}
function release(){if(gripJoint){world.removeImpulseJoint(gripJoint,true);gripJoint=null;}object?.wakeUp();}
function toggleGripper(){if(status!=='running')return;closed=!closed;if(!closed)release();$('stage').focus({preventScroll:true});updateUI();}
function setPose(body,matrix,instant=false){const p=new T.Vector3(),q=new T.Quaternion(),s=new T.Vector3();matrix.decompose(p,q,s);if(instant){body.setTranslation(p,false);body.setRotation(q,false);}else{body.setNextKinematicTranslation(p);body.setNextKinematicRotation(q);}}
// A fixed elevated view keeps the gripper, apple and bin mouth visible.
// Optical magnification never moves the eye into scene geometry.
const FIXED_VIEW={position:[1.25,-2.3,1.90],target:[.15,-.10,.70],maxZoom:1.25};
function zoomIn(factor=1.08){
 if(!camera||!Number.isFinite(factor)||factor<=1)return;
 camera.zoom=Math.min(FIXED_VIEW.maxZoom,camera.zoom*factor);
 camera.updateProjectionMatrix();
 $('zoom-in').disabled=camera.zoom>=FIXED_VIEW.maxZoom-1e-6;
 $('zoom-level').textContent=`${Math.round(camera.zoom*100)}%`;
}
function setupFixedCamera(){
 camera.position.fromArray(FIXED_VIEW.position);camera.lookAt(V(FIXED_VIEW.target));
 const canvas=$('canvas'),touches=new Map();let pinchDistance=null;
 canvas.addEventListener('wheel',event=>{
  event.preventDefault();
  if(event.deltaY<0)zoomIn(Math.exp(Math.min(-event.deltaY,120)*.0015));
 },{passive:false});
 canvas.addEventListener('pointerdown',event=>{
  $('stage').focus({preventScroll:true});
  if(event.pointerType!=='touch')return;
  touches.set(event.pointerId,[event.clientX,event.clientY]);canvas.setPointerCapture(event.pointerId);
  if(touches.size===2){const [a,b]=[...touches.values()];pinchDistance=Math.hypot(a[0]-b[0],a[1]-b[1]);}
 });
 canvas.addEventListener('pointermove',event=>{
  if(!touches.has(event.pointerId))return;
  touches.set(event.pointerId,[event.clientX,event.clientY]);
  if(touches.size!==2)return;
  const [a,b]=[...touches.values()],distance=Math.hypot(a[0]-b[0],a[1]-b[1]);
  if(pinchDistance>0&&distance>pinchDistance)zoomIn(distance/pinchDistance);
  pinchDistance=distance;
 });
 const releaseTouch=event=>{touches.delete(event.pointerId);pinchDistance=null;};
 for(const type of ['pointerup','pointercancel','lostpointercapture'])canvas.addEventListener(type,releaseTouch);
 canvas.addEventListener('contextmenu',event=>event.preventDefault());
 $('zoom-in').disabled=false;
}
function updateRobotViews(instant=false){for(const item of linkViews){item.view.matrix.copy(item.link.matrix);item.view.matrixWorldNeedsUpdate=true;}for(const item of physicsLinks)setPose(item.body,item.link.matrix,instant);setPose(handBody,robot.hand.matrix,instant);}
function reset(){
 if(!world)return;release();clearKeys();elapsed=0;motionTime=null;human='Waiting';contact=null;heldByHuman=false;humanCanHold=true;closed=false;opening=1;
 robot.reset();robot.opening(1);updateRobotViews(true);
 object.setBodyType(RAPIER.RigidBodyType.Dynamic,true);setObjectPose(objectInfo.world);
 object.setLinvel({x:0,y:0,z:0},true);object.setAngvel({x:0,y:0,z:0},true);
 avatar.skeleton.boneMatrices.set(idleSkin);avatar.skeleton.boneTexture.needsUpdate=true;capsules.set(meta.avatar.initialCapsules.flat());
 status='ready';$('score').textContent='0 / 1';syncObject();updateUI();
}
function setObjectPose(matrix){const p=new T.Vector3(),q=new T.Quaternion(),s=new T.Vector3();new T.Matrix4().fromArray(matrix).decompose(p,q,s);object.setTranslation(p,true);object.setRotation(q,true);}
function syncObject(){objectView.position.copy(object.translation());objectView.quaternion.copy(object.rotation());objectView.updateMatrix();}
function objectCenter(){
 const initial=new T.Matrix4().fromArray(objectInfo.world);
 return V(meta.objectPosition).applyMatrix4(initial.invert()).applyQuaternion(new T.Quaternion().copy(object.rotation())).add(new T.Vector3().copy(object.translation()));
}
function updateHuman(dt){
 if(motionTime===null){
  const distance=robot.tcpPosition().distanceTo(V(meta.objectPosition));
  if(elapsed<3&&distance>0.17)return;
  motionTime=0;
  humanCanHold=!gripJoint&&new T.Vector3().copy(object.translation()).distanceTo(V(meta.animation.objectPoses[0].slice(0,3)))<0.08;
 }
 motionTime=Math.min(motionTime+dt,(meta.animation.frames-1)*meta.animation.dt);
 const frame=motionTime/meta.animation.dt,a=Math.floor(frame),b=Math.min(a+1,meta.animation.frames-1),t=frame-a;
 const size=meta.avatar.bones*16;
 for(let i=0;i<size;i++)avatar.skeleton.boneMatrices[i]=skinFrames[a*size+i]*(1-t)+skinFrames[b*size+i]*t;
 avatar.skeleton.boneTexture.needsUpdate=true;
 const capSize=capsules.length;
 for(let i=0;i<capSize;i++)capsules[i]=capFrames[a*capSize+i]*(1-t)+capFrames[b*capSize+i]*t;
 human=meta.animation.phases[a];
 const attach=meta.animation.attached[a]&&humanCanHold&&!gripJoint;
 if(attach){
  if(!heldByHuman){object.setBodyType(RAPIER.RigidBodyType.KinematicPositionBased,true);heldByHuman=true;}
  const pa=meta.animation.objectPoses[a],pb=meta.animation.objectPoses[b];
  const pos=V(pa.slice(0,3)).lerp(V(pb.slice(0,3)),t),q=Q(pa.slice(3)).slerp(Q(pb.slice(3)),t);
  object.setNextKinematicTranslation(pos);object.setNextKinematicRotation(q);
 }else if(heldByHuman){
  heldByHuman=false;object.setBodyType(RAPIER.RigidBodyType.Dynamic,true);object.setLinvel({x:0,y:0,z:0},true);object.setAngvel({x:0,y:0,z:0},true);
 }
}
function graspIfAligned(){
 if(!closed||gripJoint||heldByHuman||opening>0.72)return;
 const p=new T.Vector3().copy(object.translation()),local=objectCenter().applyMatrix4(robot.tcpMatrix.clone().invert());
 // Assisted grasp is allowed only between the jaws. The resulting constraint
 // stays in the physics world; opening releases a freely falling dynamic object.
 if(Math.abs(local.x)>0.028||Math.abs(local.y)>0.032||Math.abs(local.z)>0.032)return;
 const hq=new T.Quaternion().setFromRotationMatrix(robot.hand.matrix),oq=new T.Quaternion().copy(object.rotation());
 const anchor=p.clone().applyMatrix4(robot.hand.matrix.clone().invert());
 const joint=RAPIER.JointData.fixed(anchor,hq.clone().invert().multiply(oq),{x:0,y:0,z:0},{x:0,y:0,z:0,w:1});
 gripJoint=world.createImpulseJoint(joint,handBody,object,true);
 gripJoint.setContactsEnabled(false);
 humanCanHold=false;
}
function step(){
 elapsed+=DT;
 const axis=(plus,minus)=>Number(plus.some(k=>keys.has(k)))-Number(minus.some(k=>keys.has(k)));
 const delta=new T.Vector3(axis(['KeyD','ArrowRight'],['KeyA','ArrowLeft']),axis(['KeyW','ArrowUp'],['KeyS','ArrowDown']),axis(['KeyQ'],['KeyE']));
 if(delta.lengthSq()){
  delta.normalize().multiplyScalar(DT*(keys.has('ShiftLeft')||keys.has('ShiftRight')?0.045:0.18));
  // Workspace floor prevents the kinematic tool from being driven through the table.
  const goal=robot.target.clone().add(delta);
  const minZ=goal.x<0.53&&goal.x>-.55&&goal.y>-.85&&goal.y<.16?0.765:0.13;
  if(goal.z<minZ)delta.z=Math.max(0,minZ-robot.target.z);
  if(goal.z>1.4)delta.z=0;
  robot.move(delta,DT);
 }else robot.target.copy(robot.tcpPosition());
 opening=T.MathUtils.damp(opening,closed?0.5:1,14,DT);robot.opening(opening);updateRobotViews();
 updateHuman(DT);
 contact=capsuleContact(collisionPoints,capsules,meta.avatar.capsuleNames);
 if(contact){status='failed';clearKeys();updateUI();return;}
 graspIfAligned();world.step();syncObject();
 const p=objectCenter(),inside=Math.abs(p.x-meta.binPosition[0])<meta.binHalf[0]&&Math.abs(p.y-meta.binPosition[1])<meta.binHalf[1]&&p.z>meta.floorMin&&p.z<meta.binRim+meta.rimMargin;
 $('score').textContent=inside?'1 / 1':'0 / 1';
 if(inside&&human==='Finished'&&!gripJoint){status='success';clearKeys();updateUI();}
}
async function init(){
 await RAPIER.init();meta=await getJSON('assets/scene.json?v=aligned-start-v3');
 const loaded=await Promise.all([new GLTFLoader().loadAsync('assets/scene.glb'),getBinary(meta.avatar.position),getBinary(meta.avatar.joints,Uint16Array),getBinary(meta.avatar.weights),getBinary(meta.avatar.uv),getBinary(meta.avatar.faces,Uint32Array),getBinary(meta.avatar.initialMatrices),getBinary(meta.animation.matrices),getBinary(meta.animation.capsules),new T.TextureLoader().loadAsync('assets/avatar-texture.jpg'),new T.TextureLoader().loadAsync('assets/environment.jpg?v=4k')]);
 const [gltf,position,joints,weights,uv,faces,idle,frames,caps,texture,environment]=loaded;
 renderer=new T.WebGLRenderer({canvas:$('canvas'),antialias:true,powerPreference:'high-performance'});const debugGL=renderer.getContext().getExtension("WEBGL_debug_renderer_info");const gpuName=debugGL?renderer.getContext().getParameter(debugGL.UNMASKED_RENDERER_WEBGL):"";const software=/swiftshader|llvmpipe|software/i.test(gpuName);renderer.setPixelRatio(software?.65:Math.min(devicePixelRatio,1.5));renderer.shadowMap.enabled=!software;renderer.shadowMap.type=T.PCFSoftShadowMap;renderer.toneMapping=T.ACESFilmicToneMapping;renderer.toneMappingExposure=1.3;
 scene=new T.Scene();scene.up.set(0,0,1);environment.mapping=T.EquirectangularReflectionMapping;environment.colorSpace=T.SRGBColorSpace;scene.environment=environment;scene.environmentIntensity=0.7;scene.environmentRotation.x=Math.PI/2;
 // Project the panorama onto a world-space ground at z=0. An infinitely
 // distant background has no parallax and makes the avatar appear to float.
 const room=new GroundedSkybox(environment,1.6,12,96);
 room.rotation.x=Math.PI/2;room.position.z=1.6;room.renderOrder=-10;scene.add(room);
 // Transparent shadow receiver: the photographed floor remains visible.
 const contactShadow=new T.Mesh(new T.PlaneGeometry(8,8),new T.ShadowMaterial({opacity:.16}));
 contactShadow.position.z=.002;contactShadow.receiveShadow=true;scene.add(contactShadow);
 scene.add(gltf.scene);gltf.scene.traverse(node=>{if(node.isMesh){node.castShadow=true;node.receiveShadow=true;}});
 const hemi=new T.HemisphereLight(0xf9f7ed,0x797a78,2);hemi.position.set(0,0,4);scene.add(hemi);
 const sun=new T.DirectionalLight(0xfff5e5,2.8);sun.position.set(-2,-1,5);sun.castShadow=true;sun.shadow.mapSize.set(1024,1024);sun.shadow.camera.left=-2;sun.shadow.camera.right=2;sun.shadow.camera.top=2;sun.shadow.camera.bottom=-2;sun.shadow.bias=-0.0002;sun.shadow.normalBias=0.01;scene.add(sun);
 camera=new T.PerspectiveCamera(45,1,0.02,100);camera.up.set(0,0,1);setupFixedCamera();
 const geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.BufferAttribute(position,3));geometry.setAttribute('skinIndex',new T.BufferAttribute(joints,4));geometry.setAttribute('skinWeight',new T.BufferAttribute(weights,4));geometry.setAttribute('uv',new T.BufferAttribute(uv,2));geometry.setIndex(new T.BufferAttribute(faces,1));geometry.computeVertexNormals();texture.colorSpace=T.SRGBColorSpace;texture.flipY=false;
 avatar=new T.SkinnedMesh(geometry,new T.MeshStandardMaterial({map:texture,roughness:.8,metalness:0,side:T.DoubleSide}));
 const bones=Array.from({length:meta.avatar.bones},()=>new T.Bone()),skeleton=new T.Skeleton(bones,bones.map(()=>new T.Matrix4()));skeleton.computeBoneTexture();skeleton.update=()=>{};avatar.bind(skeleton,new T.Matrix4());avatar.frustumCulled=false;avatar.castShadow=true;avatar.receiveShadow=true;scene.add(avatar);
 idleSkin=idle;skinFrames=frames;capFrames=caps;capsules=new Float32Array(meta.avatar.initialCapsules.flat());
 robot=new Robot(meta);world=new RAPIER.World({x:0,y:0,z:-9.81});world.timestep=DT;world.numSolverIterations=8;
 for(const info of meta.links){
  let view=gltf.scene.getObjectByName(info.name);
  if(!view){view=new T.Group();view.name=info.name;gltf.scene.add(view);}
  // The source scene uses an invisible floor. Keep collision geometry while
  // allowing the environment panorama to show through the display-only floor.
  view.visible=info.visible!==false;
  const matrix=new T.Matrix4().fromArray(info.world),p=new T.Vector3(),q=new T.Quaternion(),s=new T.Vector3();matrix.decompose(p,q,s);
  if(info.kind==='robot'){
   const link=robot.byName.get(info.link);view.matrixAutoUpdate=false;linkViews.push({view,link});
   const body=world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(p.x,p.y,p.z).setRotation(q));physicsLinks.push({body,link});
   for(const geom of info.colliders){
    const vertices=new Float32Array(geom.vertices.flat()),desc=RAPIER.ColliderDesc.convexHull(vertices);
    if(desc)world.createCollider(desc.setFriction(1.5).setCollisionGroups(0x00020004),body);
    const stride=Math.max(1,Math.floor(geom.vertices.length/30));const points=geom.vertices.filter((_,i)=>i%stride===0).map(V);points.push(V(geom.vertices.at(-1)));collisionPoints.push({name:info.link,points,matrix:link.matrix});
   }
  }else{
   const desc=info.kind==='object'?RAPIER.RigidBodyDesc.dynamic().setCcdEnabled(true).setLinearDamping(.12).setAngularDamping(.2):RAPIER.RigidBodyDesc.fixed();
   const body=world.createRigidBody(desc.setTranslation(p.x,p.y,p.z).setRotation(q));
   for(const geom of info.colliders){
    const vertices=new Float32Array(geom.vertices.flat());
    const collider=info.kind==='object'?RAPIER.ColliderDesc.convexHull(vertices):RAPIER.ColliderDesc.trimesh(vertices,new Uint32Array(geom.faces.flat()));
    if(collider)world.createCollider(collider.setFriction(1).setRestitution(.03).setCollisionGroups(info.kind==='object'?0x00040003:0x00010004).setDensity(350),body);
   }
   if(info.kind==='object'){
    object=body;objectView=view;objectInfo=info;
    for(const child of [...view.children])view.remove(child);
    view.add(makeApple());
   }
  }
 }
 if(!object)throw Error('Export did not include the task object.');
 handBody=world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased());
 // Read-only state is useful for reproducible browser checks. It cannot move
 // bodies, change scoring, or inject collisions.
 window.browserDemo=Object.freeze({state:()=>({status,time:elapsed,human,closed,held:!!gripJoint,object:object.translation(),objectCenter:objectCenter().toArray(),tcp:robot.tcpPosition().toArray(),collision:contact,fps:frameRate,graphics:gpuName}),source:meta.source});
 const resize=()=>{const rect=$('stage').getBoundingClientRect();renderer.setSize(rect.width,rect.height,false);camera.aspect=rect.width/rect.height;camera.fov=T.MathUtils.radToDeg(2*Math.atan(Math.tan(Math.PI/8)*Math.max(1,1.45/camera.aspect)));camera.updateProjectionMatrix();};new ResizeObserver(resize).observe($('stage'));resize();reset();
 let last=performance.now(),accumulator=0,uiTick=0;
 function animate(now){
  if(embedded&&!embeddedVisible){last=now;accumulator=0;requestAnimationFrame(animate);return;}
  const delta=Math.min((now-last)/1000,.08);last=now;
  if(status==='running'){accumulator+=delta;let steps=0;while(accumulator>=DT&&status==='running'&&steps<4){step();accumulator-=DT;steps++;}}else accumulator=0;
  renderer.render(scene,camera);frameCount++;
  if(now-lastMetric>=1000){frameRate=Math.round(frameCount*1000/(now-lastMetric));$('performance').textContent=`${frameRate} fps · local WebGL + WebAssembly`;lastMetric=now;frameCount=0;}
  if(now-uiTick>150){updateUI();uiTick=now;}requestAnimationFrame(animate);
 }
 requestAnimationFrame(animate);
}
$('start').onclick=start;$('pause').onclick=pause;$('reset').onclick=reset;$('gripper').onclick=toggleGripper;$('zoom-in').onclick=()=>zoomIn();
$('canvas').addEventListener('pointerdown',()=>$('stage').focus({preventScroll:true}));
const movement=new Set(['KeyW','KeyA','KeyS','KeyD','KeyQ','KeyE','ArrowUp','ArrowDown','ArrowLeft','ArrowRight','ShiftLeft','ShiftRight']);
document.addEventListener('keydown',event=>{if(document.activeElement!==$('stage'))return;if(movement.has(event.code)||['Space','KeyG'].includes(event.code))event.preventDefault();if(event.code==='Space'&&!event.repeat){status==='running'?pause():start();}else if(event.code==='KeyG'&&!event.repeat)toggleGripper();else if(status==='running'&&movement.has(event.code))keys.add(event.code);});
document.addEventListener('keyup',event=>keys.delete(event.code));
document.querySelectorAll('[data-key]').forEach(button=>{button.addEventListener('pointerdown',event=>{if(status!=='running')return;event.preventDefault();button.setPointerCapture(event.pointerId);$('stage').focus({preventScroll:true});keys.add(button.dataset.key);button.classList.add('active');});const release=()=>{keys.delete(button.dataset.key);button.classList.remove('active');};for(const type of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(type,release);});
window.addEventListener('blur',pause);document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});document.addEventListener('focusin',event=>{if(event.target!==$('stage'))clearKeys();});
$('canvas').addEventListener('webglcontextlost',event=>{event.preventDefault();pause();$('detail').textContent='Graphics context lost. Reload this page to restart.';});
if(embedded&&window.parent!==window){
 window.addEventListener('message',event=>{
  if(event.source!==window.parent||event.origin!==location.origin)return;
  if(event.data?.type==='genesis-demo-visibility'&&typeof event.data.visible==='boolean'){
   embeddedVisible=event.data.visible;
   if(!embeddedVisible)pause();
  }
 });
 new ResizeObserver(()=>window.parent.postMessage({type:'genesis-demo-size',height:document.body.getBoundingClientRect().height},location.origin)).observe(document.body);
}
init().catch(errorMessage);
