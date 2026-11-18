// Offline only. meshoptimizer v0.23: https://github.com/zeux/meshoptimizer/tree/v0.23/js
// Original URDF/STLs remain unchanged; only the browser's visual meshes are simplified.
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import path from 'node:path';
import {MeshoptSimplifier} from './tools/vendor/meshopt_simplifier.mjs';
const root=path.join(path.dirname(fileURLToPath(import.meta.url)),'dist');
await MeshoptSimplifier.ready;
for(const model of ['dm']){
 const manifest=JSON.parse(await readFile(path.join(root,`assets/models/${model}.json`),'utf8'));
 const meshes=new Map();let before=0,after=0,bytesBefore=0,bytesAfter=0;
 for(const visual of manifest.links.flatMap(link=>link.visuals)){
  const source=visual.mesh;
  if(!meshes.has(source)){
   const data=await readFile(path.join(root,source)),count=data.readUInt32LE(80);
   if(data.length!==84+count*50)throw Error(`Expected binary STL: ${source}`);
   const vertices=[],lookup=new Map(),indices=new Uint32Array(count*3);
   for(let face=0;face<count;face++)for(let corner=0;corner<3;corner++){
    const offset=84+face*50+12+corner*12;
    const xyz=[0,4,8].map(delta=>data.readFloatLE(offset+delta));
    const key=xyz.join(',');let index=lookup.get(key);
    if(index===undefined){index=vertices.length/3;lookup.set(key,index);vertices.push(...xyz)}
    indices[face*3+corner]=index;
   }
   const positions=new Float32Array(vertices),target=Math.min(count,Math.max(64,Math.floor(count*.12)))*3;
   const [reduced,error]=MeshoptSimplifier.simplify(indices,positions,3,target,.0015,['LockBorder']);
   const output=Buffer.alloc(84+reduced.length/3*50);
   output.write('reBot Arm web visual LOD; original CAD meshes preserved');output.writeUInt32LE(reduced.length/3,80);
   for(let face=0;face<reduced.length/3;face++){
    const points=[0,1,2].map(c=>Array.from(positions.subarray(reduced[face*3+c]*3,reduced[face*3+c]*3+3)));
    const [a,b,c]=points,u=b.map((v,i)=>v-a[i]),v=c.map((n,i)=>n-a[i]);
    const normal=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],len=Math.hypot(...normal)||1;
    [...normal.map(n=>n/len),...points.flat()].forEach((value,i)=>output.writeFloatLE(value,84+face*50+i*4));
   }
   const dest=`assets/models/optimized/${model}/${path.basename(source)}`;
   await mkdir(path.dirname(path.join(root,dest)),{recursive:true});await writeFile(path.join(root,dest),output);
   meshes.set(source,{dest,before:count,after:reduced.length/3,error});bytesBefore+=data.length;bytesAfter+=output.length;
  }
  const mesh=meshes.get(source);visual.mesh=mesh.dest;before+=mesh.before;after+=mesh.after;
 }
 manifest.webOptimization={sourceTriangles:before,triangles:after,maxRelativeError:.0015,sourceBytes:bytesBefore,bytes:bytesAfter};
 await writeFile(path.join(root,`assets/models/${model}.web.json`),JSON.stringify(manifest));
 console.log(`${model.toUpperCase()}: ${before} -> ${after} triangles; ${(bytesBefore/1e6).toFixed(2)} -> ${(bytesAfter/1e6).toFixed(2)} MB`);
}
