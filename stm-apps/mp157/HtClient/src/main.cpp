
#include <iostream>
#include <sys/types.h>
#include <unistd.h>


// Application Headers
#include "init.hpp"
#include "core_reader.hpp"


int main( int argc, char* argv[] )
{

  sysutils::SystemResetHandler::Init();

  // Can add here to run this process in a particular core for processing,
  // bcz of limited number of cores, make process run in core not thread 
  // or choose according to system needs.

  try{
    // Create Object of Application
    application::InitApplication<core::CoreReader>::Init( argc, argv);
  } catch( const std::exception& e ){
    std::cout << "Application Could not be initialized, Exiting....." << std::endl;
    exit(-1);
  }

  return 0;
}
