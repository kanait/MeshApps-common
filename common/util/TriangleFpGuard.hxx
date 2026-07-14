////////////////////////////////////////////////////////////////////
//
// Restore IEEE-754 defaults before Shewchuk Triangle runs.
//
// After Eigen / Accelerate work, macOS often leaves FPCR FZ (flush-to-
// zero) set. Triangle's exactinit() + predicates then misbehave and
// can SIGSEGV inside formskeleton. Call TriangleFpGuard for the
// duration of ::triangulate().
//
// Copyright (c) 2026 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _TRIANGLEFPGUARD_HXX
#define _TRIANGLEFPGUARD_HXX 1

#include <cfenv>
#include <cstdint>

#if defined(__SSE__)
#include <pmmintrin.h>
#include <xmmintrin.h>
#endif

namespace triangle_mesh {

class TriangleFpGuard {
 public:
  TriangleFpGuard() {
    std::fegetenv(&env_);
    std::fesetround(FE_TONEAREST);
    std::feclearexcept(FE_ALL_EXCEPT);
#if defined(__APPLE__) && defined(__aarch64__)
    // FPCR bit 24 = FZ (flush denormals to zero).
    uint64_t fpcr = 0;
    asm volatile("mrs %0, fpcr" : "=r"(fpcr));
    saved_fpcr_ = fpcr;
    constexpr uint64_t kFz = 1ull << 24;
    if (fpcr & kFz) {
      fpcr &= ~kFz;
      asm volatile("msr fpcr, %0" : : "r"(fpcr));
      cleared_fz_ = true;
    }
#elif defined(__SSE__)
    saved_mxcsr_ = _mm_getcsr();
    const unsigned cleared =
        saved_mxcsr_ & ~(_MM_FLUSH_ZERO_MASK | _MM_DENORMALS_ZERO_MASK);
    if (cleared != saved_mxcsr_) {
      _mm_setcsr(cleared);
      cleared_fz_ = true;
    }
#endif
  }

  ~TriangleFpGuard() {
#if defined(__APPLE__) && defined(__aarch64__)
    if (cleared_fz_) {
      asm volatile("msr fpcr, %0" : : "r"(saved_fpcr_));
    }
#elif defined(__SSE__)
    if (cleared_fz_) {
      _mm_setcsr(saved_mxcsr_);
    }
#endif
    std::fesetenv(&env_);
  }

  TriangleFpGuard(const TriangleFpGuard&) = delete;
  TriangleFpGuard& operator=(const TriangleFpGuard&) = delete;

 private:
  std::fenv_t env_{};
  bool cleared_fz_ = false;
#if defined(__APPLE__) && defined(__aarch64__)
  uint64_t saved_fpcr_ = 0;
#elif defined(__SSE__)
  unsigned saved_mxcsr_ = 0;
#endif
};

}  // namespace triangle_mesh

#endif  // _TRIANGLEFPGUARD_HXX
