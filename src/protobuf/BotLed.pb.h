// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: BotLed.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_BotLed_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_BotLed_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3020000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3020003 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_BotLed_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_BotLed_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_BotLed_2eproto;
namespace cas {
namespace proto {
class BotLed;
struct BotLedDefaultTypeInternal;
extern BotLedDefaultTypeInternal _BotLed_default_instance_;
}  // namespace proto
}  // namespace cas
PROTOBUF_NAMESPACE_OPEN
template<> ::cas::proto::BotLed* Arena::CreateMaybeMessage<::cas::proto::BotLed>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace cas {
namespace proto {

enum BotLed_Mode : int {
  BotLed_Mode_CLOSE = 0,
  BotLed_Mode_GREEN = 1,
  BotLed_Mode_RED = 2,
  BotLed_Mode_BotLed_Mode_INT_MIN_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::min(),
  BotLed_Mode_BotLed_Mode_INT_MAX_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::max()
};
bool BotLed_Mode_IsValid(int value);
constexpr BotLed_Mode BotLed_Mode_Mode_MIN = BotLed_Mode_CLOSE;
constexpr BotLed_Mode BotLed_Mode_Mode_MAX = BotLed_Mode_RED;
constexpr int BotLed_Mode_Mode_ARRAYSIZE = BotLed_Mode_Mode_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* BotLed_Mode_descriptor();
template<typename T>
inline const std::string& BotLed_Mode_Name(T enum_t_value) {
  static_assert(::std::is_same<T, BotLed_Mode>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function BotLed_Mode_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    BotLed_Mode_descriptor(), enum_t_value);
}
inline bool BotLed_Mode_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, BotLed_Mode* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<BotLed_Mode>(
    BotLed_Mode_descriptor(), name, value);
}
// ===================================================================

class BotLed final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:cas.proto.BotLed) */ {
 public:
  inline BotLed() : BotLed(nullptr) {}
  ~BotLed() override;
  explicit PROTOBUF_CONSTEXPR BotLed(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  BotLed(const BotLed& from);
  BotLed(BotLed&& from) noexcept
    : BotLed() {
    *this = ::std::move(from);
  }

  inline BotLed& operator=(const BotLed& from) {
    CopyFrom(from);
    return *this;
  }
  inline BotLed& operator=(BotLed&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const BotLed& default_instance() {
    return *internal_default_instance();
  }
  static inline const BotLed* internal_default_instance() {
    return reinterpret_cast<const BotLed*>(
               &_BotLed_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(BotLed& a, BotLed& b) {
    a.Swap(&b);
  }
  inline void Swap(BotLed* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(BotLed* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  BotLed* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<BotLed>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const BotLed& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const BotLed& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(BotLed* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "cas.proto.BotLed";
  }
  protected:
  explicit BotLed(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  typedef BotLed_Mode Mode;
  static constexpr Mode CLOSE =
    BotLed_Mode_CLOSE;
  static constexpr Mode GREEN =
    BotLed_Mode_GREEN;
  static constexpr Mode RED =
    BotLed_Mode_RED;
  static inline bool Mode_IsValid(int value) {
    return BotLed_Mode_IsValid(value);
  }
  static constexpr Mode Mode_MIN =
    BotLed_Mode_Mode_MIN;
  static constexpr Mode Mode_MAX =
    BotLed_Mode_Mode_MAX;
  static constexpr int Mode_ARRAYSIZE =
    BotLed_Mode_Mode_ARRAYSIZE;
  static inline const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor*
  Mode_descriptor() {
    return BotLed_Mode_descriptor();
  }
  template<typename T>
  static inline const std::string& Mode_Name(T enum_t_value) {
    static_assert(::std::is_same<T, Mode>::value ||
      ::std::is_integral<T>::value,
      "Incorrect type passed to function Mode_Name.");
    return BotLed_Mode_Name(enum_t_value);
  }
  static inline bool Mode_Parse(::PROTOBUF_NAMESPACE_ID::ConstStringParam name,
      Mode* value) {
    return BotLed_Mode_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  enum : int {
    kModeFieldNumber = 1,
  };
  // .cas.proto.BotLed.Mode mode = 1;
  void clear_mode();
  ::cas::proto::BotLed_Mode mode() const;
  void set_mode(::cas::proto::BotLed_Mode value);
  private:
  ::cas::proto::BotLed_Mode _internal_mode() const;
  void _internal_set_mode(::cas::proto::BotLed_Mode value);
  public:

  // @@protoc_insertion_point(class_scope:cas.proto.BotLed)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  int mode_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_BotLed_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// BotLed

// .cas.proto.BotLed.Mode mode = 1;
inline void BotLed::clear_mode() {
  mode_ = 0;
}
inline ::cas::proto::BotLed_Mode BotLed::_internal_mode() const {
  return static_cast< ::cas::proto::BotLed_Mode >(mode_);
}
inline ::cas::proto::BotLed_Mode BotLed::mode() const {
  // @@protoc_insertion_point(field_get:cas.proto.BotLed.mode)
  return _internal_mode();
}
inline void BotLed::_internal_set_mode(::cas::proto::BotLed_Mode value) {
  
  mode_ = value;
}
inline void BotLed::set_mode(::cas::proto::BotLed_Mode value) {
  _internal_set_mode(value);
  // @@protoc_insertion_point(field_set:cas.proto.BotLed.mode)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace cas

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::cas::proto::BotLed_Mode> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::cas::proto::BotLed_Mode>() {
  return ::cas::proto::BotLed_Mode_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_BotLed_2eproto