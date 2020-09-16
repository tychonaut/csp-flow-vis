# csp-flow-vis

Fluid flow visualization plugin for CosmoScoutVR

[Docs of main program](https://github.com/cosmoscout/cosmoscout-vr/tree/develop/docs)

TODO: for setup of infrastructure (saving Simon's local stuff to own forked and extended repositories, enable easy local testing of stuff ironically currently only workin in the cluster):
- [x] Clone, checkout and and branch "feature/csp-flow-vis" from [CosmoScoutVR fork](https://git.geomar.de/arena/cosmoscout-vr.git).
- [x] fix error "fatal: remote error: upload-pack: not our ref ac62f2fcfda3feaf0a2fb926f723ad1215087771"
      (reason was a local bugfix commit I couldn"t push to a remote. Hope that Simons fix-commit will also work.)
- [ ] commit and push initial plugin source-files from Simon
- [ ] find where to define install dir for cmake, maybe create wrapper build script around make.bat for main program
- [ ] build branch of main program according to [generic build instructions](https://github.com/cosmoscout/cosmoscout-vr/blob/develop/docs/install.md) 
- [ ] perform local test run with default setup
- [ ] understand **cluster** config files in "flow_vis_cluster" profile:
- [ ] 1. GEBCO DEM stuff
- [ ] 2. map-cache'd stuff (helgoland, mars ..) and their definition in config files
- [ ] 3. space navigator
- [ ] 4. pointer tracking (xbox  controller)
- [ ] extract local-single-instance config files + start script 
      from existing  **cluster** configs ("flow_vis_cluster" profile)
- [ ] perform local test run with GEBCO stuff
- [ ] perform local test runs with map-cache'd stuff (helgoland, mars ..)
- [ ] MultiOSCluster: implement mechanism for config-copy from profile folder (MultiOSCluster repo) 
      to local installation folder, analogous to 
      `./appControl/rsyncAppConfigToCluster.sh CosmoScoutVR`
      extend existing script functionality, then create small wrapper script
- [ ] set up visual studio code accourding to CS doc


TODO: recreate minimalistic initial plugin functionality that worked locally on Simon's ubuntu laptop:
- [ ] write this list

TODO: implement the actual functionality from the rough framework
- [ ] write this list






