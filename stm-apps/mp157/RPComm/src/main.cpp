
#include <iostream>
#include <string_view>
#include <memory>
#include <vector>
#include <algorithm>
#include <numeric>
#include <list>
#include <array>
#include <chrono>
#include <cstddef>
#include <iomanip>

#include <sys/types.h>
#include <unistd.h>

// pmr
#include <pmr_headers.hpp>
#include <monotonic_buffer_resource.hpp>

// shared memory
#include <shared_memory.hpp>

// thread pool
#include <thread_pool.hpp>



class debug_resource : public pmr::memory_resource {
  public:
    explicit debug_resource(std::string name, pmr::memory_resource* up = pmr::get_default_resource())
		: _name{ std::move(name) }, _upstream{ up } 
	  { }

    void* do_allocate(size_t bytes, size_t alignment) override {
      std::cout << _name << " do_allocate(): " << bytes << '\n';
      void* ret = _upstream->allocate(bytes, alignment);
      return ret;
	  }

	  void do_deallocate(void* ptr, size_t bytes, size_t alignment) override {
      std::cout << _name << " do_deallocate(): " << bytes << '\n';
      _upstream->deallocate(ptr, bytes, alignment);
    }

    bool do_is_equal(const pmr::memory_resource& other) const noexcept override {
		  return this == &other;
	  }
    
  private:
    std::string _name;
    pmr::memory_resource* _upstream;
};


template <typename Func>
auto benchmark(Func test_func, int iterations) {
  const auto start = std::chrono::system_clock::now();
  while (iterations-- > 0) { test_func(); }
  const auto stop = std::chrono::system_clock::now();
  const auto secs = std::chrono::duration<double>(stop - start);
  return secs.count();
}

void do_pmr_benchmark()
{
  constexpr int iterations{100};
  constexpr int total_nodes{2'00'000};

  auto default_std_alloc = [total_nodes]
  {
    std::list<int> list;
    for (int i{}; i != total_nodes; ++i)
    {
      list.push_back(i);
    }
  };

  auto default_pmr_alloc = [total_nodes]
  {
    pmr::list<int> list;
    for (int i{}; i != total_nodes; ++i)
    {
      list.push_back(i);
    }
  };

  auto pmr_alloc_no_buf = [total_nodes]
  {
    pmr::monotonic_buffer_resource mbr(nullptr);
    pmr::polymorphic_allocator<int> pa{&mbr};
    pmr::list<int> list{pa};
    for (int i{}; i != total_nodes; ++i)
    {
      list.push_back(i);
    }
  };

  auto pmr_alloc_and_buf = [total_nodes]
  {
    std::array<pmr::byte, total_nodes * 32> buffer; // enough to fit in all nodes
    pmr::monotonic_buffer_resource mbr{buffer.data(), buffer.size()};
    pmr::polymorphic_allocator<int> pa{&mbr};
    pmr::list<int> list{pa};
    for (int i{}; i != total_nodes; ++i)
    {
      list.push_back(i);
    }
  };

  const double t1 = benchmark(default_std_alloc, iterations);
  const double t2 = benchmark(default_pmr_alloc, iterations);
  const double t3 = benchmark(pmr_alloc_no_buf, iterations);
  const double t4 = benchmark(pmr_alloc_and_buf, iterations);

  std::cout << std::fixed << std::setprecision(3)
            << "t1 (default std alloc): " << t1 << " sec; t1/t1: " << t1 / t1 << '\n'
            << "t2 (default pmr alloc): " << t2 << " sec; t1/t2: " << t1 / t2 << '\n'
            << "t3 (pmr alloc  no buf): " << t3 << " sec; t1/t3: " << t1 / t3 << '\n'
            << "t4 (pmr alloc and buf): " << t4 << " sec; t1/t4: " << t1 / t4 << '\n';
}


const std::string dataWrite{"Hello Server, I am Client using SHM for data transfer"};

struct DataShm
{
  DataShm( int xi, int yi, std::string stri )
  : x{xi}, y{yi}, str{stri}
  {}

  DataShm()
  : x{0}, y{0}, str{}
  {}

  int x;
  int y;
  std::string str;
};


// Write data to SHM
DataShm dataWriteStr{5, 4, "Hello World"};


void ShmClienThread(void)
{
  std::cout << "Entered Client SHM function\n";

  // Cretae Named Semaphore
  utils::Semaphore semaphoreInterface_(std::string("/testSem"));

  // Open Shared Memory as Writer/Client
  utils::SharedMemory<DataShm> shm(std::string("/testShm"), 4096U, semaphoreInterface_);

  int32_t status = shm.map(0);
  if( status ){
    std::cout << "Shared Memory Creation Failed, Exiting\n";
    return;
  }

  if( 0 == shm.write(dataWriteStr, sizeof(dataWriteStr)) ){
    std::cout << "Data written to SHM successfully with size = " << sizeof(dataWriteStr) << std::endl;
  }

  // Post signal for Reader
  if( -1 == shm.getSemaphore()->post() ){
    std::cout << "Could not POST using Semaphore" << std::endl;
  }

  if( shm.unmap() ){
    std::cout << "SHM UnMapping Failed" << std::endl;
  }

  void * ptr = shm.get_shm_ptr();
  if(nullptr == ptr){
    std::cout << "SHM Writer Pointer NULL" << std::endl;
  }

  std::cout << "Exiting Client SHM function\n";
}


int main( int argc, char* argv[] )
{
  #ifdef PMR_BENCHMARK
    do_pmr_benchmark();
  #endif

  // Cretae Named Semaphore
  utils::Semaphore semaphoreInterface_(std::string("/testSem"), utils::SemaphoreI::SemaphoreFlags::SEM_CREATE, 0);

{
  // Open Shared Memory as Reader/Server
  utils::SharedMemory<DataShm> shm(std::string("/testShm"), 4096U, semaphoreInterface_,
                                        utils::SharedMemoryI::ShmFlags::SHM_CREATE_RDWR);

  int32_t status = shm.map(0);
  if( status ){
    std::cout << "Shared Memory Creation Failed, Exiting\n";
    return -1;
  }

{
  // Create Client Thread to write data to SHM
  utils::ThreadPool threadPool(1, std::string("ThreadPool"));

  // Create Job for SHM
  threadPool.init();

  threadPool.postJob(ShmClienThread);
}

  std::cout << "Waiting for Semaphore signal from client\n";
  // Wait for signal to read data
  if( -1 == shm.getSemaphore()->wait() ){
    std::cout << "Could not Wait using Semaphore" << std::endl;
  }

  // Read data from SHM
  DataShm dataReadStr{};

  if( 0 == shm.read(dataReadStr, 40) ){
    std::cout << "Data read from SHM is " << dataReadStr.x << " " << dataReadStr.y << " " << dataReadStr.str << std::endl;
  }

  // Unmap Memory and destroy it
  if( shm.unmap() ){
    std::cout << "SHM UnMapping Failed" << std::endl;
    return -1;
  }

  // Check shared memory pointer status
  void * ptr = shm.get_shm_ptr();
  if(nullptr == ptr){
    std::cout << "SHM Reader Pointer NULL" << std::endl;
  }
}

  return 0;
}
