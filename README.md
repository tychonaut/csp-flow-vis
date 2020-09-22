# csp-flow-vis

Fluid flow visualization plugin for CosmoScoutVR

[Docs of main program](https://github.com/cosmoscout/cosmoscout-vr/tree/develop/docs)

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
- [ ] set up visual studio code accourding to CS doc

TODO: recreate minimalistic initial plugin functionality that worked locally on Simon's ubuntu laptop:
- [ ] check data-read-in of csp-flow-vis: point to D:\app_data\global\CosmoScoutVR\plugin-data\csp-flow-vis\flow-data-simon
      via flow_vis.json -> deserialize -> ...
- [x] check how to create plugin- local cmake and generate appropriate header and shared libs
- [ ] remember/lookup how to use plugin in main app, how to configure stuff, where to put data used by plugin
      (csp-flow-vis is dervied from csp-simple-bodies --> config json will be similar, look at source code)
- [ ] run local test of plugin
- [ ] develop mechanism (rsync or scp) to efficiently copy app asset/resource data from develop or master machine to cluster:
       experiment with rsync-deletion-flags, maybe eventually NAS (though windows remote shell policies may be a problem) 
- [ ] adapt cluster config for csp-flow-vis plugin
- [ ] upload binaries and config to dome
- [ ] test run in dome

TODO: implement the actual functionality from the rough framework
- [ ] determine specifi TODOs from insights gathered from above steps, then write this list
- [ ] ...
- [ ] test run in dome





## Configuration

This plugin can be enabled with the following configuration in your `settings.json`:

```javascript
{
  ...
  "plugins": {
    ...
    "csp-flow-vis": {
      "TODO...": {
        <anchor name>: {
          "texture": <path to surface texture>
        },
        ... <more bodies> ...
      }
    }
  }
}
```





