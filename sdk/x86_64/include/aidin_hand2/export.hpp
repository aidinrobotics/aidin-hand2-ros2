
#ifndef AIDIN_HAND2_EXPORT_H
#define AIDIN_HAND2_EXPORT_H

#ifdef AIDIN_HAND2_STATIC_DEFINE
#  define AIDIN_HAND2_EXPORT
#  define AIDIN_HAND2_NO_EXPORT
#else
#  ifndef AIDIN_HAND2_EXPORT
#    ifdef aidin_hand2_objects_EXPORTS
        /* We are building this library */
#      define AIDIN_HAND2_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define AIDIN_HAND2_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef AIDIN_HAND2_NO_EXPORT
#    define AIDIN_HAND2_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef AIDIN_HAND2_DEPRECATED
#  define AIDIN_HAND2_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef AIDIN_HAND2_DEPRECATED_EXPORT
#  define AIDIN_HAND2_DEPRECATED_EXPORT AIDIN_HAND2_EXPORT AIDIN_HAND2_DEPRECATED
#endif

#ifndef AIDIN_HAND2_DEPRECATED_NO_EXPORT
#  define AIDIN_HAND2_DEPRECATED_NO_EXPORT AIDIN_HAND2_NO_EXPORT AIDIN_HAND2_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef AIDIN_HAND2_NO_DEPRECATED
#    define AIDIN_HAND2_NO_DEPRECATED
#  endif
#endif

#endif /* AIDIN_HAND2_EXPORT_H */
