# csp-flow-vis

Fluid flow visualization plugin for CosmoScoutVR.

<p align="center"> 
  <img src ="resources/screenshots/csp-flow-vis_prototype1_shot2.png" />
</p>



[Docs of main program](https://github.com/cosmoscout/cosmoscout-vr/tree/develop/docs)


TODOs after demo:

- [x] hack first 2+1D flow visualization prototype for demo
- [x] upload unlisted youtube video of dome presentation
- [x] make this list
- [x] clean up worst mess behind me: browser and text file tabs, update VS etc.
- [x] check how to best integrate a screenshot into MD file
- [x] mirror to public github repository

- [x] ask administration about working times
- [x] thank DLR guys for their support so far, send screenshot and youtube video (ask supervisor for permission first), 
      ask about xbox controller support for CSVR, GUI
- [x] contact tat team about availability
- [ ] thank Klaus & Torge for their support, show video, ask about:

      - correctness of my approach: 
        location of vectors on sphere, 
        orientation of vectors on sphere, 
        units the vectors encode (meters/second?), 
        correct relative velocities on each point on the sphere, 
        strange apparent bias for horiz/vertical flow ("tiling"): discretization artifacts or visualization error?
        ellipsoid vs sphere
      - routines to establish correctness,
      - provision of 3+1D data (ask supervisor which datasets)

- [ ] more3D cluster mode: check out  doc & ideas

- [ ] talk to supervisor about next steps and priorities: 

      - more3d cluster mode
      - Correctness & color coding
      - performance optimization: (deterministic noise on GPU instead of readback-CPUnoise-upload), 
      - different animation time scales: seconds vs weeks between animation frames; think about elegant resolution and interpolation
      - Cleanup & refactoring, 
      - useability optimization: State safe&restore,
      - useability optimization: GUI, 
      - 2+1D-->3+1D (which data, which approach), 
      - Riccardo's Track plugin, data for it (AUVs, ROVs, fish swarms)
      - xbox wand + buttons --> ask simon for update, tat team @ intuitive controller,
      - another bathmetry dataset (GEBCO2020, GMRT?)
      - schedule meeting about future work modalidies: home and presence office etc.

- [ ] make acquainted with GUI programming, maybe ask Simon for support
- [ ] ...









## Configuration

This plugin can be enabled with the following configuration in your `settings.json`:

```javascript
{
  ...
  "plugins": {
    ...
    "csp-flow-vis": {
	"isEnabled": true,
	"tifDirectory": "D:/app_data/global/CosmoScoutVR/plugin-data/csp-flow-vis/flow-data-simon",
	"anchor": "Earth",
	"startDate": "2019-01-01 00:00:00.000",
	"endDate": "2019-12-31 00:00:00.000",		
	"west": -100.461669921875, 
	"east": 32.311336517333984, 
	"south": -33.9864501953125, 
	"north": 69.38933563232422,
	"numTimeSteps" : 72,
	"flowSpeedScale" : 0.01,
	"particleSeedThreshold": 0.001
    },
    ...
  }
}
```

## Older TODOs

TODO: for setup of infrastructure (saving Simon's local stuff to own forked and extended repositories, enable easy local testing of stuff ironically currently only workin in the cluster):
- [x] Clone, checkout and and branch "feature/csp-flow-vis" from [CosmoScoutVR fork](https://git.geomar.de/arena/cosmoscout-vr.git).
- [x] fix error "fatal: remote error: upload-pack: not our ref ac62f2fcfda3feaf0a2fb926f723ad1215087771"
      (reason was a local bugfix commit I couldn"t push to a remote. Hope that Simons fix-commit will also work.)
- [x] commit and push initial plugin source-files from Simon
- [x] find where to define install dir for cmake, maybe create wrapper build script around make.bat for main program
- [x] build branch of main program according to [generic build instructions](https://github.com/cosmoscout/cosmoscout-vr/blob/develop/docs/install.md) 
- [x] perform local test run with default setup
- [x] understand **cluster** config files in "flow_vis_cluster" profile:
- [x] 1. GEBCO DEM stuff
- [x] 2. map-cache'd stuff (helgoland, mars ..) and their definition in config files
- [x] 3. space navigator
- [ ] 4. pointer tracking (xbox  controller)
- [x] extract local-single-instance config files + start script 
      from existing  **cluster** configs ("flow_vis_cluster" profile)
- [x] perform local test run with GEBCO stuff
- [x] perform local test runs with map-cache'd stuff (helgoland, mars ..)
- [x] create graphs and notes about compilation/deployment/config-syncing/launch
- [x] create graphs and notes about config and asset/resource data
- [x] MultiOSCluster: implement mechanism for config-copy from profile folder (MultiOSCluster repo) 
      to local installation folder, analogous to 
      `./appControl/rsyncAppConfigToCluster.sh CosmoScoutVR`
      extend existing script functionality, then create small wrapper script
- [x] set up visual studio  accourding to CS doc

TODO: recreate minimalistic initial plugin functionality that worked locally on Simon's ubuntu laptop:
- [x] check data-read-in of csp-flow-vis: point to D:\app_data\global\CosmoScoutVR\plugin-data\csp-flow-vis\flow-data-simon
      via flow_vis.json -> deserialize -> ...
- [x] check how to create plugin- local cmake and generate appropriate header and shared libs
- [x] remember/lookup how to use plugin in main app, how to configure stuff, where to put data used by plugin
      (csp-flow-vis is dervied from csp-simple-bodies --> config json will be similar, look at source code)
- [x] run local test of plugin
- [x] develop mechanism (rsync or scp) to efficiently copy app asset/resource data from develop or master machine to cluster:
- [ ] experiment with rsync-deletion-flags, maybe eventually NAS (though windows remote shell policies may be a problem) 
- [x] adapt cluster config for csp-flow-vis plugin
- [x] upload binaries and config to dome
- [x] test run in dome
- [x] checkout implications of correction from  
      `gdaladdo step$STEP.tif 2 4 8` to 
      `gdaladdo step_$STEP.tif 2 4 8`: expecting mipmaps, elping in webservers, but not for opengl,
      which has to build its own mip maps, right?

TODO: implement the actual flow visualization functionality from the rough framework
- [x] implement first hacky prototype 
- [x] test run in dome





