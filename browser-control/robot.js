import * as T from 'three';
const V = a => new T.Vector3().fromArray(a);
const Q = a => new T.Quaternion(a[1], a[2], a[3], a[0]);
const clamp = T.MathUtils.clamp;
// Small damped least-squares solve. Damping keeps near-singular robot poses bounded.
function solve(a, b) {
  const rows = a.map((r, i) => [...r, b[i]]), n = b.length;
  for (let k = 0; k < n; k++) {
    let pivot = k;
    for (let i = k + 1; i < n; i++) if (Math.abs(rows[i][k]) > Math.abs(rows[pivot][k])) pivot = i;
    [rows[k], rows[pivot]] = [rows[pivot], rows[k]];
    const div = rows[k][k];
    if (Math.abs(div) < 1e-12) return Array(n).fill(0);
    for (let j = k; j <= n; j++) rows[k][j] /= div;
    for (let i = 0; i < n; i++) if (i !== k) {
      const s = rows[i][k];
      for (let j = k; j <= n; j++) rows[i][j] -= s * rows[k][j];
    }
  }
  return rows.map(r => r[n]);
}
export class Robot {
  constructor(meta) {
    this.links = meta.robot.map(r => ({...r, matrix: new T.Matrix4().fromArray(r.world), initial: new T.Matrix4().fromArray(r.world)}));
    this.byIndex = new Map(this.links.map(l => [l.index, l]));
    this.byName = new Map(this.links.map(l => [l.name, l]));
    this.joints = [];
    for (const link of this.links) {
      link.parentLink = this.byIndex.get(link.parent);
      link.rest = link.parentLink ? link.parentLink.initial.clone().invert().multiply(link.initial) : link.initial.clone();
      link.joint = link.joints.find(j => j.q.length === 1);
      if (link.joint) {
        const j = link.joint;
        j.initial = j.q[0]; j.value = j.initial;
        j.axisV = V(j.axis[0] || [0,0,0]); j.linearV = V(j.linear[0] || [0,0,0]); j.origin = V(j.pos);
        if (/^joint[1-7]$/.test(j.name)) this.joints.push({link, j});
      }
    }
    this.hand = this.byName.get(meta.eeLink);
    this.tcpOffset = new T.Matrix4().compose(V(meta.tcp.slice(0,3)), Q(meta.tcp.slice(3)), new T.Vector3(1,1,1));
    this.forward(); this.target = this.tcpPosition(); this.orientation = this.tcpQuaternion();
  }
  forward() {
    for (const link of this.links) {
      link.matrix.copy(link.rest);
      const j = link.joint;
      if (j) {
        const dq = j.value - j.initial;
        if (j.axisV.lengthSq() > 0) {
          link.matrix.multiply(new T.Matrix4().makeTranslation(...j.origin.toArray()));
          link.matrix.multiply(new T.Matrix4().makeRotationAxis(j.axisV, dq));
          link.matrix.multiply(new T.Matrix4().makeTranslation(...j.origin.clone().negate().toArray()));
        } else link.matrix.multiply(new T.Matrix4().makeTranslation(...j.linearV.clone().multiplyScalar(dq).toArray()));
      }
      if (link.parentLink) link.matrix.premultiply(link.parentLink.matrix);
    }
    this.tcpMatrix = this.hand.matrix.clone().multiply(this.tcpOffset);
  }
  tcpPosition() { return new T.Vector3().setFromMatrixPosition(this.tcpMatrix); }
  tcpQuaternion() { return new T.Quaternion().setFromRotationMatrix(this.tcpMatrix); }
  reset() { for (const l of this.links) if (l.joint) l.joint.value = l.joint.initial; this.forward(); this.target.copy(this.tcpPosition()); }
  opening(value) { for (const l of this.links) if (l.joint?.name.startsWith('finger_joint')) l.joint.value = value * 0.04; this.forward(); }
  move(delta, dt) {
    const before = this.joints.map(({j}) => j.value);
    this.target.add(delta);
    // Keep unreachable targets close enough that releasing a key stops motion promptly.
    const offset = this.target.clone().sub(this.tcpPosition());
    if (offset.length() > 0.025) this.target.copy(this.tcpPosition()).add(offset.setLength(0.025));
    for (let iteration = 0; iteration < 5; iteration++) {
      const p = this.tcpPosition(), orientation = this.tcpQuaternion();
      const dq = this.orientation.clone().multiply(orientation.clone().invert());
      if (dq.w < 0) dq.set(-dq.x,-dq.y,-dq.z,-dq.w);
      const error = [...this.target.clone().sub(p).toArray(), dq.x * 0.6, dq.y * 0.6, dq.z * 0.6];
      const cols = [];
      for (const {j} of this.joints) {
        j.value += 0.0001; this.forward();
        const pp = this.tcpPosition().sub(p).multiplyScalar(10000);
        const qq = this.tcpQuaternion().multiply(orientation.clone().invert());
        cols.push([...pp.toArray(), qq.x * 6000, qq.y * 6000, qq.z * 6000]);
        j.value -= 0.0001;
      }
      const a = Array.from({length:6},(_,r)=>Array.from({length:6},(_,c)=>cols.reduce((s,j)=>s+j[r]*j[c],0)+(r===c?0.00015:0)));
      const x = solve(a,error);
      this.joints.forEach(({j},i)=>{
        const step = cols[i].reduce((sum,c,k)=>sum+c*x[k],0);
        const low = Math.max(j.limits[0][0], before[i] - dt * 0.8), high = Math.min(j.limits[0][1], before[i] + dt * 0.8);
        j.value = clamp(j.value + clamp(step,-0.06,0.06),low,high);
      });
      this.forward();
    }
  }
}
// Same capsule radii and geometry-point samples as the exported task, with a
// conservative capsule test. The native checker's forearm skin refinement is
// not reproduced, so this browser collision result is not a benchmark score.
export function capsuleContact(robotPoints, capsuleData, names) {
  for (const group of robotPoints) {
    const worldPoints = group.points.map(p => p.clone().applyMatrix4(group.matrix));
    for (let c=0;c<names.length;c++) {
      const k=c*7, ax=capsuleData[k], ay=capsuleData[k+1], az=capsuleData[k+2];
      const dx=capsuleData[k+3]-ax, dy=capsuleData[k+4]-ay, dz=capsuleData[k+5]-az, r=capsuleData[k+6];
      const length2=dx*dx+dy*dy+dz*dz;
      for (const p of worldPoints) {
        const t=clamp(((p.x-ax)*dx+(p.y-ay)*dy+(p.z-az)*dz)/Math.max(length2,1e-12),0,1);
        const distance=Math.hypot(p.x-ax-t*dx,p.y-ay-t*dy,p.z-az-t*dz)-r;
        if(distance < 0) return {link:group.name, human:names[c], penetration:-distance};
      }
    }
  }
  return null;
}
