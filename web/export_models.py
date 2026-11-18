"""Export the repository's URDF and referenced visual meshes for the web viewer."""
from pathlib import Path
import json
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / 'src/rebotarm_bringup/description'
OUT = ROOT / 'web/dist/assets/models'


def vector(value, default='0 0 0'):
    return [float(v) for v in (value or default).split()]


def export(model):
    urdf = DESCRIPTION / model / 'urdf' / f'ReBot_Arm_{model}.urdf'
    tree = ET.parse(urdf).getroot()
    materials = {m.attrib['name']: vector(m.find('color').get('rgba')) for m in tree.findall('material') if m.find('color') is not None}
    links, joints = [], []
    folder = OUT / model.lower()
    folder.mkdir(parents=True, exist_ok=True)
    copied = set()
    for link in tree.findall('link'):
        visuals = []
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            filename = mesh.get('filename')
            source = (ROOT / 'src/rebotarm_bringup' / filename.removeprefix('package://rebotarm_bringup/')) if filename.startswith('package://') else (urdf.parent / filename).resolve()
            target = folder / source.name
            if source not in copied:
                shutil.copyfile(source, target)
                copied.add(source)
            origin = visual.find('origin')
            mat = visual.find('material')
            color = materials.get(mat.get('name'), [0.65, 0.68, 0.64, 1]) if mat is not None else [0.65, 0.68, 0.64, 1]
            if mat is not None and mat.find('color') is not None:
                color = vector(mat.find('color').get('rgba'))
            visuals.append({'mesh': f'assets/models/{model.lower()}/{source.name}', 'xyz':vector(origin.get('xyz') if origin is not None else None), 'rpy':vector(origin.get('rpy') if origin is not None else None), 'color': color, 'scale': vector(mesh.get('scale'), '1 1 1')})
        links.append({'name':link.get('name'), 'visuals':visuals})
    for joint in tree.findall('joint'):
        origin, axis, limit, mimic = joint.find('origin'), joint.find('axis'), joint.find('limit'), joint.find('mimic')
        joints.append({'name':joint.get('name'), 'type':joint.get('type'), 'parent':joint.find('parent').get('link'), 'child':joint.find('child').get('link'), 'xyz':vector(origin.get('xyz') if origin is not None else None), 'rpy':vector(origin.get('rpy') if origin is not None else None), 'axis':vector(axis.get('xyz') if axis is not None else None), 'lower':float(limit.get('lower',0)) if limit is not None else 0, 'upper':float(limit.get('upper',0)) if limit is not None else 0, 'mimic':dict(mimic.attrib) if mimic is not None else None})
    (OUT / f'{model.lower()}.json').write_text(json.dumps({'model':model, 'links':links,'joints':joints},separators=(',',':')))
    print(f'{model}: {len(links)} links, {len(joints)} joints, {len(copied)} meshes')


if __name__ == '__main__':
    for model in ('DM',):
        export(model)
