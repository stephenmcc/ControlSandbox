plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   group = "us.ihmc"
   version = "0.11.0"
   vcsUrl = "https://stash.ihmc.us/scm/rob/valkyrieuserinterface"
   openSource = false
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:valkyrie:source")
   api("org.hipparchus:hipparchus-optim:2.3")
}

