import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const assets=new URL('../dist/',import.meta.url);
const bounds=data=>{const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];const count=data.readUInt32LE(80);assert.equal(data.length,84+count*50);for(let face=0;face<count;face++)for(let corner=0;corner<3;corner++)for(let axis=0;axis<3;axis++){const value=data.readFloatLE(84+face*50+12+corner*12+axis*4);assert.ok(Number.isFinite(value));min[axis]=Math.min(min[axis],value);max[axis]=Math.max(max[axis],value)}return{min,max,count}};
for(const model of ['dm'])test(`${model}: lighter visuals preserve URDF kinematics, material groups and mesh extents`,async()=>{
 const original=JSON.parse(await readFile(new URL(`assets/models/${model}.json`,assets),'utf8'));
 const optimized=JSON.parse(await readFile(new URL(`assets/models/${model}.web.json`,assets),'utf8'));
 assert.deepEqual(optimized.joints,original.joints);
 assert.equal(optimized.links.length,original.links.length);
 let triangles=0,oldTriangles=0;
 for(let i=0;i<original.links.length;i++){
  const a=original.links[i],b=optimized.links[i];assert.equal(a.name,b.name);assert.equal(a.visuals.length,b.visuals.length);
  for(let j=0;j<a.visuals.length;j++){
   const {mesh:oldMesh,...oldProperties}=a.visuals[j],{mesh:newMesh,...newProperties}=b.visuals[j];assert.deepEqual(newProperties,oldProperties);
   const source=bounds(await readFile(new URL(oldMesh,assets))),web=bounds(await readFile(new URL(newMesh,assets)));
   triangles+=web.count;oldTriangles+=source.count;assert.ok(web.count>0);
   const extent=Math.max(...source.max.map((value,k)=>value-source.min[k]));
   for(let k=0;k<3;k++){assert.ok(Math.abs(source.min[k]-web.min[k])<=extent*.002+1e-6,`${newMesh}: min bound`);assert.ok(Math.abs(source.max[k]-web.max[k])<=extent*.002+1e-6,`${newMesh}: max bound`)}
  }
 }
 assert.ok(triangles<oldTriangles*.3);assert.equal(triangles,optimized.webOptimization.triangles);
});
