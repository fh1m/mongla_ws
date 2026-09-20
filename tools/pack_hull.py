import pickle,numpy as np,struct,gzip
parts=pickle.load(open('parts.pkl','rb'))
V=[];F=[];off=0
for n,v,f in parts:
    V.append(v); F.append(f+off); off+=len(v)
V=np.vstack(V).astype(np.float64); F=np.vstack(F)
print("in  tris",len(F),"verts",len(V))
LO,HI=V.min(0),V.max(0); C=(LO+HI)/2; S=(HI-LO).max()/2
def build(cell):
    q=np.floor((V-LO)/cell).astype(np.int64)
    key=q[:,0]*1_000_003+q[:,1]*1009+q[:,2]
    uniq,inv=np.unique(key,return_inverse=True)
    nv=np.zeros((len(uniq),3))
    np.add.at(nv,inv,V); cnt=np.bincount(inv,minlength=len(uniq))
    nv/=cnt[:,None]
    nf=inv[F]
    ok=(nf[:,0]!=nf[:,1])&(nf[:,1]!=nf[:,2])&(nf[:,0]!=nf[:,2])
    return nv,nf[ok]
for cell in (0.0009,):
    nv,nf=build(cell)
    if len(nv)<65536:
        print(f"cell {cell*1000:.1f}mm -> verts {len(nv)} tris {len(nf)}")

print("out tris",len(nf),"verts",len(nv))
qp=np.clip(np.round((nv-C)/S*32767),-32767,32767).astype('<i2')
idx=nf.astype('<u2') if len(nv)<65536 else nf.astype('<u4')
hdr=struct.pack('<4sHHII3f f',b'MGLA',1,(2 if idx.itemsize==2 else 4),len(nv),len(nf),*C,S)
blob=hdr+qp.tobytes()+idx.tobytes()
open('/home/fh1m/Envs/dockers/auv-ros2/Ros_workspaces/duburi_ws/docs/assets/cad/hull.mgla','wb').write(blob)
print("raw bytes",len(blob),"gz",len(gzip.compress(blob,9)))
