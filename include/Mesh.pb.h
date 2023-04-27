// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Mesh.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_Mesh_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_Mesh_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3019000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3019004 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_Mesh_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_Mesh_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[4]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_Mesh_2eproto;
namespace cas {
namespace proto {
class Mesh;
struct MeshDefaultTypeInternal;
extern MeshDefaultTypeInternal _Mesh_default_instance_;
class V1;
struct V1DefaultTypeInternal;
extern V1DefaultTypeInternal _V1_default_instance_;
class V2;
struct V2DefaultTypeInternal;
extern V2DefaultTypeInternal _V2_default_instance_;
class V3;
struct V3DefaultTypeInternal;
extern V3DefaultTypeInternal _V3_default_instance_;
}  // namespace proto
}  // namespace cas
PROTOBUF_NAMESPACE_OPEN
template<> ::cas::proto::Mesh* Arena::CreateMaybeMessage<::cas::proto::Mesh>(Arena*);
template<> ::cas::proto::V1* Arena::CreateMaybeMessage<::cas::proto::V1>(Arena*);
template<> ::cas::proto::V2* Arena::CreateMaybeMessage<::cas::proto::V2>(Arena*);
template<> ::cas::proto::V3* Arena::CreateMaybeMessage<::cas::proto::V3>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace cas {
namespace proto {

// ===================================================================

class Mesh final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:cas.proto.Mesh) */ {
 public:
  inline Mesh() : Mesh(nullptr) {}
  ~Mesh() override;
  explicit constexpr Mesh(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Mesh(const Mesh& from);
  Mesh(Mesh&& from) noexcept
    : Mesh() {
    *this = ::std::move(from);
  }

  inline Mesh& operator=(const Mesh& from) {
    CopyFrom(from);
    return *this;
  }
  inline Mesh& operator=(Mesh&& from) noexcept {
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
  static const Mesh& default_instance() {
    return *internal_default_instance();
  }
  static inline const Mesh* internal_default_instance() {
    return reinterpret_cast<const Mesh*>(
               &_Mesh_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Mesh& a, Mesh& b) {
    a.Swap(&b);
  }
  inline void Swap(Mesh* other) {
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
  void UnsafeArenaSwap(Mesh* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Mesh* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Mesh>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Mesh& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const Mesh& from);
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
  void InternalSwap(Mesh* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "cas.proto.Mesh";
  }
  protected:
  explicit Mesh(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kV1FieldNumber = 1,
    kV2FieldNumber = 2,
    kV3FieldNumber = 3,
    kRFieldNumber = 4,
    kGFieldNumber = 5,
    kBFieldNumber = 6,
  };
  // repeated .cas.proto.V1 v1 = 1;
  int v1_size() const;
  private:
  int _internal_v1_size() const;
  public:
  void clear_v1();
  ::cas::proto::V1* mutable_v1(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V1 >*
      mutable_v1();
  private:
  const ::cas::proto::V1& _internal_v1(int index) const;
  ::cas::proto::V1* _internal_add_v1();
  public:
  const ::cas::proto::V1& v1(int index) const;
  ::cas::proto::V1* add_v1();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V1 >&
      v1() const;

  // repeated .cas.proto.V2 v2 = 2;
  int v2_size() const;
  private:
  int _internal_v2_size() const;
  public:
  void clear_v2();
  ::cas::proto::V2* mutable_v2(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V2 >*
      mutable_v2();
  private:
  const ::cas::proto::V2& _internal_v2(int index) const;
  ::cas::proto::V2* _internal_add_v2();
  public:
  const ::cas::proto::V2& v2(int index) const;
  ::cas::proto::V2* add_v2();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V2 >&
      v2() const;

  // repeated .cas.proto.V3 v3 = 3;
  int v3_size() const;
  private:
  int _internal_v3_size() const;
  public:
  void clear_v3();
  ::cas::proto::V3* mutable_v3(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V3 >*
      mutable_v3();
  private:
  const ::cas::proto::V3& _internal_v3(int index) const;
  ::cas::proto::V3* _internal_add_v3();
  public:
  const ::cas::proto::V3& v3(int index) const;
  ::cas::proto::V3* add_v3();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V3 >&
      v3() const;

  // repeated float r = 4;
  int r_size() const;
  private:
  int _internal_r_size() const;
  public:
  void clear_r();
  private:
  float _internal_r(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_r() const;
  void _internal_add_r(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_r();
  public:
  float r(int index) const;
  void set_r(int index, float value);
  void add_r(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      r() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_r();

  // repeated float g = 5;
  int g_size() const;
  private:
  int _internal_g_size() const;
  public:
  void clear_g();
  private:
  float _internal_g(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_g() const;
  void _internal_add_g(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_g();
  public:
  float g(int index) const;
  void set_g(int index, float value);
  void add_g(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      g() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_g();

  // repeated float b = 6;
  int b_size() const;
  private:
  int _internal_b_size() const;
  public:
  void clear_b();
  private:
  float _internal_b(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_b() const;
  void _internal_add_b(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_b();
  public:
  float b(int index) const;
  void set_b(int index, float value);
  void add_b(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      b() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_b();

  // @@protoc_insertion_point(class_scope:cas.proto.Mesh)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V1 > v1_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V2 > v2_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V3 > v3_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > r_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > g_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > b_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_Mesh_2eproto;
};
// -------------------------------------------------------------------

class V1 final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:cas.proto.V1) */ {
 public:
  inline V1() : V1(nullptr) {}
  ~V1() override;
  explicit constexpr V1(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  V1(const V1& from);
  V1(V1&& from) noexcept
    : V1() {
    *this = ::std::move(from);
  }

  inline V1& operator=(const V1& from) {
    CopyFrom(from);
    return *this;
  }
  inline V1& operator=(V1&& from) noexcept {
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
  static const V1& default_instance() {
    return *internal_default_instance();
  }
  static inline const V1* internal_default_instance() {
    return reinterpret_cast<const V1*>(
               &_V1_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(V1& a, V1& b) {
    a.Swap(&b);
  }
  inline void Swap(V1* other) {
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
  void UnsafeArenaSwap(V1* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  V1* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<V1>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const V1& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const V1& from);
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
  void InternalSwap(V1* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "cas.proto.V1";
  }
  protected:
  explicit V1(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kXFieldNumber = 1,
    kYFieldNumber = 2,
    kZFieldNumber = 3,
  };
  // float x = 1;
  void clear_x();
  float x() const;
  void set_x(float value);
  private:
  float _internal_x() const;
  void _internal_set_x(float value);
  public:

  // float y = 2;
  void clear_y();
  float y() const;
  void set_y(float value);
  private:
  float _internal_y() const;
  void _internal_set_y(float value);
  public:

  // float z = 3;
  void clear_z();
  float z() const;
  void set_z(float value);
  private:
  float _internal_z() const;
  void _internal_set_z(float value);
  public:

  // @@protoc_insertion_point(class_scope:cas.proto.V1)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  float x_;
  float y_;
  float z_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_Mesh_2eproto;
};
// -------------------------------------------------------------------

class V2 final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:cas.proto.V2) */ {
 public:
  inline V2() : V2(nullptr) {}
  ~V2() override;
  explicit constexpr V2(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  V2(const V2& from);
  V2(V2&& from) noexcept
    : V2() {
    *this = ::std::move(from);
  }

  inline V2& operator=(const V2& from) {
    CopyFrom(from);
    return *this;
  }
  inline V2& operator=(V2&& from) noexcept {
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
  static const V2& default_instance() {
    return *internal_default_instance();
  }
  static inline const V2* internal_default_instance() {
    return reinterpret_cast<const V2*>(
               &_V2_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  friend void swap(V2& a, V2& b) {
    a.Swap(&b);
  }
  inline void Swap(V2* other) {
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
  void UnsafeArenaSwap(V2* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  V2* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<V2>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const V2& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const V2& from);
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
  void InternalSwap(V2* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "cas.proto.V2";
  }
  protected:
  explicit V2(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kXFieldNumber = 1,
    kYFieldNumber = 2,
    kZFieldNumber = 3,
  };
  // float x = 1;
  void clear_x();
  float x() const;
  void set_x(float value);
  private:
  float _internal_x() const;
  void _internal_set_x(float value);
  public:

  // float y = 2;
  void clear_y();
  float y() const;
  void set_y(float value);
  private:
  float _internal_y() const;
  void _internal_set_y(float value);
  public:

  // float z = 3;
  void clear_z();
  float z() const;
  void set_z(float value);
  private:
  float _internal_z() const;
  void _internal_set_z(float value);
  public:

  // @@protoc_insertion_point(class_scope:cas.proto.V2)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  float x_;
  float y_;
  float z_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_Mesh_2eproto;
};
// -------------------------------------------------------------------

class V3 final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:cas.proto.V3) */ {
 public:
  inline V3() : V3(nullptr) {}
  ~V3() override;
  explicit constexpr V3(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  V3(const V3& from);
  V3(V3&& from) noexcept
    : V3() {
    *this = ::std::move(from);
  }

  inline V3& operator=(const V3& from) {
    CopyFrom(from);
    return *this;
  }
  inline V3& operator=(V3&& from) noexcept {
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
  static const V3& default_instance() {
    return *internal_default_instance();
  }
  static inline const V3* internal_default_instance() {
    return reinterpret_cast<const V3*>(
               &_V3_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    3;

  friend void swap(V3& a, V3& b) {
    a.Swap(&b);
  }
  inline void Swap(V3* other) {
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
  void UnsafeArenaSwap(V3* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  V3* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<V3>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const V3& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const V3& from);
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
  void InternalSwap(V3* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "cas.proto.V3";
  }
  protected:
  explicit V3(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kXFieldNumber = 1,
    kYFieldNumber = 2,
    kZFieldNumber = 3,
  };
  // float x = 1;
  void clear_x();
  float x() const;
  void set_x(float value);
  private:
  float _internal_x() const;
  void _internal_set_x(float value);
  public:

  // float y = 2;
  void clear_y();
  float y() const;
  void set_y(float value);
  private:
  float _internal_y() const;
  void _internal_set_y(float value);
  public:

  // float z = 3;
  void clear_z();
  float z() const;
  void set_z(float value);
  private:
  float _internal_z() const;
  void _internal_set_z(float value);
  public:

  // @@protoc_insertion_point(class_scope:cas.proto.V3)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  float x_;
  float y_;
  float z_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_Mesh_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Mesh

// repeated .cas.proto.V1 v1 = 1;
inline int Mesh::_internal_v1_size() const {
  return v1_.size();
}
inline int Mesh::v1_size() const {
  return _internal_v1_size();
}
inline void Mesh::clear_v1() {
  v1_.Clear();
}
inline ::cas::proto::V1* Mesh::mutable_v1(int index) {
  // @@protoc_insertion_point(field_mutable:cas.proto.Mesh.v1)
  return v1_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V1 >*
Mesh::mutable_v1() {
  // @@protoc_insertion_point(field_mutable_list:cas.proto.Mesh.v1)
  return &v1_;
}
inline const ::cas::proto::V1& Mesh::_internal_v1(int index) const {
  return v1_.Get(index);
}
inline const ::cas::proto::V1& Mesh::v1(int index) const {
  // @@protoc_insertion_point(field_get:cas.proto.Mesh.v1)
  return _internal_v1(index);
}
inline ::cas::proto::V1* Mesh::_internal_add_v1() {
  return v1_.Add();
}
inline ::cas::proto::V1* Mesh::add_v1() {
  ::cas::proto::V1* _add = _internal_add_v1();
  // @@protoc_insertion_point(field_add:cas.proto.Mesh.v1)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V1 >&
Mesh::v1() const {
  // @@protoc_insertion_point(field_list:cas.proto.Mesh.v1)
  return v1_;
}

// repeated .cas.proto.V2 v2 = 2;
inline int Mesh::_internal_v2_size() const {
  return v2_.size();
}
inline int Mesh::v2_size() const {
  return _internal_v2_size();
}
inline void Mesh::clear_v2() {
  v2_.Clear();
}
inline ::cas::proto::V2* Mesh::mutable_v2(int index) {
  // @@protoc_insertion_point(field_mutable:cas.proto.Mesh.v2)
  return v2_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V2 >*
Mesh::mutable_v2() {
  // @@protoc_insertion_point(field_mutable_list:cas.proto.Mesh.v2)
  return &v2_;
}
inline const ::cas::proto::V2& Mesh::_internal_v2(int index) const {
  return v2_.Get(index);
}
inline const ::cas::proto::V2& Mesh::v2(int index) const {
  // @@protoc_insertion_point(field_get:cas.proto.Mesh.v2)
  return _internal_v2(index);
}
inline ::cas::proto::V2* Mesh::_internal_add_v2() {
  return v2_.Add();
}
inline ::cas::proto::V2* Mesh::add_v2() {
  ::cas::proto::V2* _add = _internal_add_v2();
  // @@protoc_insertion_point(field_add:cas.proto.Mesh.v2)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V2 >&
Mesh::v2() const {
  // @@protoc_insertion_point(field_list:cas.proto.Mesh.v2)
  return v2_;
}

// repeated .cas.proto.V3 v3 = 3;
inline int Mesh::_internal_v3_size() const {
  return v3_.size();
}
inline int Mesh::v3_size() const {
  return _internal_v3_size();
}
inline void Mesh::clear_v3() {
  v3_.Clear();
}
inline ::cas::proto::V3* Mesh::mutable_v3(int index) {
  // @@protoc_insertion_point(field_mutable:cas.proto.Mesh.v3)
  return v3_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V3 >*
Mesh::mutable_v3() {
  // @@protoc_insertion_point(field_mutable_list:cas.proto.Mesh.v3)
  return &v3_;
}
inline const ::cas::proto::V3& Mesh::_internal_v3(int index) const {
  return v3_.Get(index);
}
inline const ::cas::proto::V3& Mesh::v3(int index) const {
  // @@protoc_insertion_point(field_get:cas.proto.Mesh.v3)
  return _internal_v3(index);
}
inline ::cas::proto::V3* Mesh::_internal_add_v3() {
  return v3_.Add();
}
inline ::cas::proto::V3* Mesh::add_v3() {
  ::cas::proto::V3* _add = _internal_add_v3();
  // @@protoc_insertion_point(field_add:cas.proto.Mesh.v3)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::cas::proto::V3 >&
Mesh::v3() const {
  // @@protoc_insertion_point(field_list:cas.proto.Mesh.v3)
  return v3_;
}

// repeated float r = 4;
inline int Mesh::_internal_r_size() const {
  return r_.size();
}
inline int Mesh::r_size() const {
  return _internal_r_size();
}
inline void Mesh::clear_r() {
  r_.Clear();
}
inline float Mesh::_internal_r(int index) const {
  return r_.Get(index);
}
inline float Mesh::r(int index) const {
  // @@protoc_insertion_point(field_get:cas.proto.Mesh.r)
  return _internal_r(index);
}
inline void Mesh::set_r(int index, float value) {
  r_.Set(index, value);
  // @@protoc_insertion_point(field_set:cas.proto.Mesh.r)
}
inline void Mesh::_internal_add_r(float value) {
  r_.Add(value);
}
inline void Mesh::add_r(float value) {
  _internal_add_r(value);
  // @@protoc_insertion_point(field_add:cas.proto.Mesh.r)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
Mesh::_internal_r() const {
  return r_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
Mesh::r() const {
  // @@protoc_insertion_point(field_list:cas.proto.Mesh.r)
  return _internal_r();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
Mesh::_internal_mutable_r() {
  return &r_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
Mesh::mutable_r() {
  // @@protoc_insertion_point(field_mutable_list:cas.proto.Mesh.r)
  return _internal_mutable_r();
}

// repeated float g = 5;
inline int Mesh::_internal_g_size() const {
  return g_.size();
}
inline int Mesh::g_size() const {
  return _internal_g_size();
}
inline void Mesh::clear_g() {
  g_.Clear();
}
inline float Mesh::_internal_g(int index) const {
  return g_.Get(index);
}
inline float Mesh::g(int index) const {
  // @@protoc_insertion_point(field_get:cas.proto.Mesh.g)
  return _internal_g(index);
}
inline void Mesh::set_g(int index, float value) {
  g_.Set(index, value);
  // @@protoc_insertion_point(field_set:cas.proto.Mesh.g)
}
inline void Mesh::_internal_add_g(float value) {
  g_.Add(value);
}
inline void Mesh::add_g(float value) {
  _internal_add_g(value);
  // @@protoc_insertion_point(field_add:cas.proto.Mesh.g)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
Mesh::_internal_g() const {
  return g_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
Mesh::g() const {
  // @@protoc_insertion_point(field_list:cas.proto.Mesh.g)
  return _internal_g();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
Mesh::_internal_mutable_g() {
  return &g_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
Mesh::mutable_g() {
  // @@protoc_insertion_point(field_mutable_list:cas.proto.Mesh.g)
  return _internal_mutable_g();
}

// repeated float b = 6;
inline int Mesh::_internal_b_size() const {
  return b_.size();
}
inline int Mesh::b_size() const {
  return _internal_b_size();
}
inline void Mesh::clear_b() {
  b_.Clear();
}
inline float Mesh::_internal_b(int index) const {
  return b_.Get(index);
}
inline float Mesh::b(int index) const {
  // @@protoc_insertion_point(field_get:cas.proto.Mesh.b)
  return _internal_b(index);
}
inline void Mesh::set_b(int index, float value) {
  b_.Set(index, value);
  // @@protoc_insertion_point(field_set:cas.proto.Mesh.b)
}
inline void Mesh::_internal_add_b(float value) {
  b_.Add(value);
}
inline void Mesh::add_b(float value) {
  _internal_add_b(value);
  // @@protoc_insertion_point(field_add:cas.proto.Mesh.b)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
Mesh::_internal_b() const {
  return b_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
Mesh::b() const {
  // @@protoc_insertion_point(field_list:cas.proto.Mesh.b)
  return _internal_b();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
Mesh::_internal_mutable_b() {
  return &b_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
Mesh::mutable_b() {
  // @@protoc_insertion_point(field_mutable_list:cas.proto.Mesh.b)
  return _internal_mutable_b();
}

// -------------------------------------------------------------------

// V1

// float x = 1;
inline void V1::clear_x() {
  x_ = 0;
}
inline float V1::_internal_x() const {
  return x_;
}
inline float V1::x() const {
  // @@protoc_insertion_point(field_get:cas.proto.V1.x)
  return _internal_x();
}
inline void V1::_internal_set_x(float value) {
  
  x_ = value;
}
inline void V1::set_x(float value) {
  _internal_set_x(value);
  // @@protoc_insertion_point(field_set:cas.proto.V1.x)
}

// float y = 2;
inline void V1::clear_y() {
  y_ = 0;
}
inline float V1::_internal_y() const {
  return y_;
}
inline float V1::y() const {
  // @@protoc_insertion_point(field_get:cas.proto.V1.y)
  return _internal_y();
}
inline void V1::_internal_set_y(float value) {
  
  y_ = value;
}
inline void V1::set_y(float value) {
  _internal_set_y(value);
  // @@protoc_insertion_point(field_set:cas.proto.V1.y)
}

// float z = 3;
inline void V1::clear_z() {
  z_ = 0;
}
inline float V1::_internal_z() const {
  return z_;
}
inline float V1::z() const {
  // @@protoc_insertion_point(field_get:cas.proto.V1.z)
  return _internal_z();
}
inline void V1::_internal_set_z(float value) {
  
  z_ = value;
}
inline void V1::set_z(float value) {
  _internal_set_z(value);
  // @@protoc_insertion_point(field_set:cas.proto.V1.z)
}

// -------------------------------------------------------------------

// V2

// float x = 1;
inline void V2::clear_x() {
  x_ = 0;
}
inline float V2::_internal_x() const {
  return x_;
}
inline float V2::x() const {
  // @@protoc_insertion_point(field_get:cas.proto.V2.x)
  return _internal_x();
}
inline void V2::_internal_set_x(float value) {
  
  x_ = value;
}
inline void V2::set_x(float value) {
  _internal_set_x(value);
  // @@protoc_insertion_point(field_set:cas.proto.V2.x)
}

// float y = 2;
inline void V2::clear_y() {
  y_ = 0;
}
inline float V2::_internal_y() const {
  return y_;
}
inline float V2::y() const {
  // @@protoc_insertion_point(field_get:cas.proto.V2.y)
  return _internal_y();
}
inline void V2::_internal_set_y(float value) {
  
  y_ = value;
}
inline void V2::set_y(float value) {
  _internal_set_y(value);
  // @@protoc_insertion_point(field_set:cas.proto.V2.y)
}

// float z = 3;
inline void V2::clear_z() {
  z_ = 0;
}
inline float V2::_internal_z() const {
  return z_;
}
inline float V2::z() const {
  // @@protoc_insertion_point(field_get:cas.proto.V2.z)
  return _internal_z();
}
inline void V2::_internal_set_z(float value) {
  
  z_ = value;
}
inline void V2::set_z(float value) {
  _internal_set_z(value);
  // @@protoc_insertion_point(field_set:cas.proto.V2.z)
}

// -------------------------------------------------------------------

// V3

// float x = 1;
inline void V3::clear_x() {
  x_ = 0;
}
inline float V3::_internal_x() const {
  return x_;
}
inline float V3::x() const {
  // @@protoc_insertion_point(field_get:cas.proto.V3.x)
  return _internal_x();
}
inline void V3::_internal_set_x(float value) {
  
  x_ = value;
}
inline void V3::set_x(float value) {
  _internal_set_x(value);
  // @@protoc_insertion_point(field_set:cas.proto.V3.x)
}

// float y = 2;
inline void V3::clear_y() {
  y_ = 0;
}
inline float V3::_internal_y() const {
  return y_;
}
inline float V3::y() const {
  // @@protoc_insertion_point(field_get:cas.proto.V3.y)
  return _internal_y();
}
inline void V3::_internal_set_y(float value) {
  
  y_ = value;
}
inline void V3::set_y(float value) {
  _internal_set_y(value);
  // @@protoc_insertion_point(field_set:cas.proto.V3.y)
}

// float z = 3;
inline void V3::clear_z() {
  z_ = 0;
}
inline float V3::_internal_z() const {
  return z_;
}
inline float V3::z() const {
  // @@protoc_insertion_point(field_get:cas.proto.V3.z)
  return _internal_z();
}
inline void V3::_internal_set_z(float value) {
  
  z_ = value;
}
inline void V3::set_z(float value) {
  _internal_set_z(value);
  // @@protoc_insertion_point(field_set:cas.proto.V3.z)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace cas

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_Mesh_2eproto