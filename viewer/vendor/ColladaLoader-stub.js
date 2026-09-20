// URDFLoader imports ColladaLoader at module level. This viewer loads STL only, and the real
// ColladaLoader drags in ColladaComposer, ColladaParser and TGALoader, so it is stubbed out
// and mapped here by the import map in index.html.
//
// If a mesh in DAE ever appears in the description, this file has to become the real loader
// and its three dependencies have to be vendored beside it.
export class ColladaLoader {
  constructor() {
    throw new Error('this viewer loads STL meshes only, DAE is not supported');
  }
}
