#pragma once

#include <cstddef>
#include <vector>
#include <concepts>
#include <memory>
#include <string>
#include <sstream>
#include <unordered_map>

namespace bsk {
    struct mismatched_schemas_error : public std::runtime_error {
        using std::runtime_error::runtime_error;

        mismatched_schemas_error()
            : std::runtime_error("mismatched schemas")
        {}
    };

    struct unsupported_focus_error : public std::runtime_error {
        using std::runtime_error::runtime_error;

        unsupported_focus_error()
            : std::runtime_error("unsupported focus type")
        {}
    };

    struct illegal_focus_error : public std::runtime_error {
        using std::runtime_error::runtime_error;

        illegal_focus_error()
            : std::runtime_error("illegal focus")
        {}
    };

    struct plug {
        virtual ~plug() {}

        virtual std::string repr() const = 0;

        virtual std::size_t focusable_indices() const {
            return 0; // no/zero focusable indices
        }

        virtual std::vector<std::string> focusable_names() const {
            return {}; // no/zero focusable names
        }

        // struct field focusing
        virtual std::shared_ptr<plug> focus(std::string const name [[maybe_unused]]) const {
            throw unsupported_focus_error();
        }

        // array element focusing
        virtual std::shared_ptr<plug> focus(std::size_t const index [[maybe_unused]]) const {
            throw unsupported_focus_error();
        }

        // array slice focusing
        virtual std::shared_ptr<plug> focus(std::size_t const start [[maybe_unused]], std::size_t const length [[maybe_unused]]) const {
            throw unsupported_focus_error();
        }
    };

    struct socket {
        virtual ~socket() {}

        virtual std::string repr() const = 0;

        // may throw mismatched_schemas_error
        virtual void subscribeTo(plug const& source) = 0;

        virtual bool can_subscribe_to(plug const& source) const = 0;

        virtual std::size_t focusable_indices() const {
            return 0; // no/zero focusable indices
        }

        virtual std::vector<std::string> focusable_names() const {
            return {}; // no/zero focusable names
        }

        // struct field focusing
        virtual std::shared_ptr<socket> focus(std::string const name [[maybe_unused]]) {
            throw unsupported_focus_error();
        }

        // array element focusing
        virtual std::shared_ptr<socket> focus(std::size_t const index [[maybe_unused]]) {
            throw unsupported_focus_error();
        }

        // array slice focusing
        virtual std::shared_ptr<socket> focus(std::size_t const start [[maybe_unused]], std::size_t const length [[maybe_unused]]) {
            throw unsupported_focus_error();
        }
    };

    // metadata
    struct message_header;

    template<typename T>
    concept is_plug = requires {
        std::derived_from<T, plug>;
    };

    template<typename T>
    concept is_socket = requires {
        std::derived_from<T, socket>;
    };

    template<typename T>
    concept is_schema = requires {
        typename T::carrier;
        is_plug<typename T::plug>;
        is_socket<typename T::socket>;
    };
}

namespace bsk {
    // Adapted from <https://stackoverflow.com/a/68523970>.
    //
    // The constants in this function depend on the length of this function's
    // declaration in the source code. Don't change/rename it unless you also
    // double-check the constants! (Just use `std::cout << type_of<int>()`
    // or something similar, and count the number of extraneous characters
    // on either side of the desired substring.)
    template <typename U>
    static consteval std::string_view type_of() {
        std::string_view name = __PRETTY_FUNCTION__;
        name.remove_prefix(37);
        name.remove_suffix(1);
        return name;
    }

    /*
    ** A compile-time registry of schemas for given types,
    ** with a default of opaque<T> for otherwise-unknown types.
    ** Specializations introduce custom schemas for specific types.
    **
    ** This design is inspired by the "Lucky 7" of ztd
    **   https://ztdtext.readthedocs.io/en/latest/design/lucky%207.html#lucky-7
    ** although the type registry `schema<T>` is an extra layer on top.
    */
    template<typename T>
    struct schema final {
        using carrier = T;

        static std::string repr() {
            return std::string(type_of<carrier>());
        }

        struct plug final : public bsk::plug {
            message_header const* const header;
            carrier const* const payload;

            plug(message_header const* const header, carrier const* const payload)
                : header(header)
                , payload(payload)
            {}

            std::string repr() const override {
                return schema::repr();
            }
        };

        struct socket final : public bsk::socket {
            message_header const** const header;
            carrier const** const payload;

            socket(message_header const** const header, carrier const** const payload)
                : header(header)
                , payload(payload)
            {}

            void subscribeTo(plug const& source) {
                *this->header = source.header;
                *this->payload = source.payload;
            }

            void subscribeTo(bsk::plug const& other) override {
                auto source = dynamic_cast<plug const*>(&other);
                if (source == nullptr) throw mismatched_schemas_error();
                return this->subscribeTo(*source);
            }

            bool can_subscribe_to(bsk::plug const& other) const override {
                auto source = dynamic_cast<plug const*>(&other);
                if (source == nullptr) return false;
                return true;
            }

            std::string repr() const override {
                return schema::repr();
            }
        };
    };

    template<typename T>
    using carrier_for = typename schema<T>::carrier;

    template<typename T>
    using plug_for = typename schema<T>::plug;

    template<typename T>
    using socket_for = typename schema<T>::socket;


    template<typename T>
    struct schema<T[]> final {
        using carrier = T;
        using element_plug = plug_for<T>;

        static std::string repr(size_t length) {
            return schema<T>::repr() + "[" + std::to_string(length) + "]";
        }

        struct plug final : public bsk::plug {
            message_header const* const header;
            carrier const* const payload;
            std::size_t const length;

            plug(message_header const* const header, carrier const* const payload, std::size_t const length)
                : header(header)
                , payload(payload)
                , length(length)
            {}

            constexpr element_plug pick(std::size_t const index) const {
                if (this->length <= index) throw illegal_focus_error();

                return element_plug(this->header, &this->payload[index]);
            }

            constexpr plug slice(std::size_t const start, std::size_t const length) const {
                if (this->length <= start) throw illegal_focus_error();
                if (this->length - start < length) throw illegal_focus_error();

                return plug(this->header, &this->payload[start], length);
            }

            std::size_t focusable_indices() const override {
                return length;
            }

            std::shared_ptr<bsk::plug> focus(std::size_t const index) const override {
                return std::make_shared<element_plug>(this->pick(index));
            }

            std::shared_ptr<bsk::plug> focus(std::size_t start, std::size_t length) const override {
                return std::make_shared<plug>(this->slice(start, length));
            }

            std::string repr() const override {
                return schema::repr(this->length);
            }
        };

        struct socket final : public bsk::socket {
            message_header const** const header;
            carrier const** const payload;
            std::size_t const length;

            socket(message_header const** const header, carrier const** const payload, std::size_t const length)
                : header(header)
                , payload(payload)
                , length(length)
            {}

            void subscribeTo(plug const& source) {
                if (this->length != source.length) throw mismatched_schemas_error();

                *this->payload = source.payload;
                *this->header = source.header;
            }

            void subscribeTo(bsk::plug const& other) override {
                auto source = dynamic_cast<plug const*>(&other);
                if (source == nullptr) throw mismatched_schemas_error();
                return this->subscribeTo(*source);
            }

            bool can_subscribe_to(bsk::plug const& other) const override {
                auto source = dynamic_cast<plug const*>(&other);
                if (source == nullptr) return false;
                return true;
            }

            std::string repr() const override {
                return schema::repr(this->length);
            }
        };
    };

    template<typename T, size_t N>
    struct schema<T[N]> final {
        // `T[N] const` is the same as `T const[N]`,
        // since `const` distributes through array declarations.
        using carrier = T[N];
        using element_plug = plug_for<T>;
        using slice_plug = plug_for<T[]>;
        using slice_socket = plug_for<T[]>;

        static std::string repr() {
            return schema<T>::repr() + "[" + std::to_string(N) + "]";
        }

        struct plug final : public bsk::plug {
            message_header const* const header;
            carrier const* const payload;

            plug(message_header const* const header, carrier const* const payload)
                : header(header)
                , payload(payload)
            {}

            constexpr element_plug pick(std::size_t const index) const {
                if (N <= index) throw illegal_focus_error();

                return element_plug(this->header, &(*this->payload)[index]);
            }

            constexpr slice_plug slice(std::size_t const start, std::size_t const length) const {
                if (N <= start) throw illegal_focus_error();
                if (N - start < length) throw illegal_focus_error();

                return slice_plug(this->header, &(*this->payload)[start], length);
            }

            std::size_t focusable_indices() const override {
                return N;
            }

            std::shared_ptr<bsk::plug> focus(std::size_t const index) const override {
                return std::make_shared<element_plug>(this->pick(index));
            }

            std::shared_ptr<bsk::plug> focus(std::size_t start, std::size_t length) const override {
                return std::make_shared<slice_plug>(this->slice(start, length));
            }

            std::string repr() const override {
                return schema::repr();
            }
        };

        struct socket final : public bsk::socket {
            message_header const** const header;
            carrier const** const payload;

            socket(message_header const** const header, carrier const** const payload)
                : header(header)
                , payload(payload)
            {}

            void subscribeTo(plug const& source) {
                *this->payload = source.payload;
                *this->header = source.header;
            }

            void subscribeTo(slice_plug const& source) {
                if (N != source.length) throw mismatched_schemas_error();

                // TODO: Is there another way to line up a pointer-to-T[N] with a pointer-to-T?
                *this->payload = reinterpret_cast<carrier const*>(source.payload);
            }

            void subscribeTo(bsk::plug const& other) override {
                if (auto source = dynamic_cast<plug const*>(&other); source != nullptr) {
                    return this->subscribeTo(*source);
                } else if (auto source = dynamic_cast<slice_plug const*>(&other); source != nullptr) {
                    return this->subscribeTo(*source);
                } else {
                    throw mismatched_schemas_error();
                }
            }

            bool can_subscribe_to(bsk::plug const& other) const override {
                if (auto source = dynamic_cast<plug const*>(&other); source != nullptr) {
                    return true;
                } else if (auto source = dynamic_cast<slice_plug const*>(&other); source != nullptr) {
                    return true;
                } else {
                    return false;
                }
            }

            std::string repr() const override {
                return schema::repr();
            }
        };
    };

    static_assert(bsk::is_schema<bsk::schema<int>>);
    static_assert(bsk::is_schema<bsk::schema<int[]>>);
    static_assert(bsk::is_schema<bsk::schema<int[42]>>);
}

namespace bsk {
    struct message_header {
        // TODO: add metadata
    };

    template<typename T>
    class message final {
    private:
        T payload;
        message_header header = {};

    public:
        template<typename ...Args>
        message(Args&&... args)
            : payload {args...}
        {}

        T& operator*() {
            // TODO: update header dirty/write information
            return this->payload;
        }

        std::shared_ptr<bsk::plug> to_shared_plug() const {
            return std::make_shared<plug_for<T>>(&this->header, &this->payload);
        }
    };

    template<typename T>
    class read_functor final {
    private:
        T const* payload = nullptr;
        message_header const* header = nullptr;

    public:
        bool isLinked() const {
            return (this->payload != nullptr);
        }

        T const& operator*() const {
            return *this->payload;
        }

        T const&& getOrElse(T const&& alternative) const {
            return (this->isLinked())
                ? std::move(**this)
                : std::forward<T const>(alternative);
        }

        std::shared_ptr<socket> to_shared_socket() {
            return std::make_shared<socket_for<T>>(&this->header, &this->payload);
        }
    };
}

namespace bsk {
    class outputs final : public plug {
    private:
        std::unordered_map<std::string, std::shared_ptr<plug>> components;

    public:
        struct component {
            std::string key;
            std::shared_ptr<plug> value;

            template<typename T>
            component(std::string&& key, bsk::message<T> const& value);

            component(std::string&& key, bsk::outputs&& value);
        };


        outputs(std::initializer_list<component> components)
            : components(components.size())
        {
            for (auto component : components) {
                this->components[component.key] = component.value;
            }
        }

        std::shared_ptr<bsk::plug> to_shared_plug() && {
            return std::make_shared<outputs>(std::move(*this));
        }

        void insert(std::string const& key, std::shared_ptr<plug> value) {
            this->components[key] = value;
        }

        decltype(components)::const_iterator begin() const {
            return this->components.cbegin();
        }

        decltype(components)::const_iterator end() const {
            return this->components.cend();
        }

        std::vector<std::string> focusable_names() const override {
            // // C++20, clang 14+
            // auto keys = std::views::keys(this->components);
            // return {keys.begin(), keys.end()};

            std::vector<std::string> keys;
            keys.reserve(this->components.size());

            for (auto const& entry : this->components) {
                keys.push_back(entry.first);
            }

            return keys;
        }

        std::shared_ptr<plug> focus(std::string const key) const override {
            try {
                return this->components.at(key);
            } catch (std::out_of_range& ex) {
                throw illegal_focus_error();
            }
        }

        std::string repr() const override {
            auto iter = this->components.cbegin();
            auto end = this->components.cend();
            if (iter == end) return "bundle {}";

            std::stringstream buf;
            buf << "bundle { ";

            for (; iter != end; ++iter) {
                buf << "\"" << iter->first << "\": " << iter->second->repr() << ", ";
            }

            // overwrite the trailing ", "
            buf.seekp(-2, std::ios_base::cur);
            buf << " }";

            return std::move(buf).str();
        }
    };

    template<typename T>
    outputs::component::component(std::string&& key, bsk::message<T> const& value)
        : key(key), value(value.to_shared_plug())
    {}

    outputs::component::component(std::string&& key, bsk::outputs&& value)
        : key(key), value(std::move(value).to_shared_plug())
    {}


    class inputs final : public socket {
    private:
        std::unordered_map<std::string, std::shared_ptr<socket>> components;

    public:
        struct component {
            std::string key;
            std::shared_ptr<socket> value;

            template<typename T>
            component(std::string&& key, bsk::read_functor<T>& value);

            component(std::string&& key, bsk::inputs&& value);
        };


        inputs(std::initializer_list<component> components)
            : components(components.size())
        {
            for (auto&& component : components) {
                this->components[component.key] = std::move(component.value);
            }
        }

        std::shared_ptr<bsk::socket> to_shared_socket() && {
            return std::make_shared<inputs>(std::move(*this));
        }

        void insert(std::string const& key, std::shared_ptr<socket> value) {
            this->components[key] = value;
        }

        decltype(components)::const_iterator begin() const {
            return this->components.cbegin();
        }

        decltype(components)::const_iterator end() const {
            return this->components.cend();
        }

        void subscribeTo(plug const& source) override {
            try {
                for (auto const& entry : this->components) {
                    entry.second->subscribeTo(*source.focus(entry.first));
                }
            } catch (illegal_focus_error const& ex) {
                throw mismatched_schemas_error();
            } catch (unsupported_focus_error const& ex) {
                throw unsupported_focus_error();
            }
        }

        bool can_subscribe_to(plug const& source) const override {
            for (auto const& entry : this->components) {
                if (!entry.second->can_subscribe_to(*source.focus(entry.first))) {
                    return false;
                }
            }
            return true;
        }

        std::vector<std::string> focusable_names() const override {
            // // C++20, clang 14+
            // auto keys = std::views::keys(this->components);
            // return {keys.begin(), keys.end()};

            std::vector<std::string> keys;
            keys.reserve(this->components.size());

            for (auto const& entry : this->components) {
                keys.push_back(entry.first);
            }

            return keys;
        }

        std::shared_ptr<socket> focus(std::string const key) override {
            try {
                return this->components.at(key);
            } catch (std::out_of_range& ex) {
                throw illegal_focus_error();
            }
        }

        std::string repr() const override {
            auto iter = this->components.cbegin();
            auto end = this->components.cend();
            if (iter == end) return "bundle {}";

            std::stringstream buf;
            buf << "bundle { ";

            for (; iter != end; ++iter) {
                buf << "\"" << iter->first << "\": " << iter->second->repr() << ", ";
            }

            // overwrite the trailing ", "
            buf.seekp(-2, std::ios_base::cur);
            buf << " }";

            return std::move(buf).str();
        }
    };

    template<typename T>
    inputs::component::component(std::string&& key, bsk::read_functor<T>& value)
        : key(key), value(value.to_shared_socket())
    {}

    inputs::component::component(std::string&& key, bsk::inputs&& value)
        : key(key), value(std::move(value).to_shared_socket())
    {}


    class fanout final : public socket {
    private:
        std::vector<std::shared_ptr<socket>> targets;

    public:
        fanout(std::initializer_list<std::shared_ptr<socket>> targets)
            : targets(targets)
        {}

        fanout(std::vector<std::shared_ptr<socket>> targets)
            : targets(targets)
        {}

        void subscribeTo(plug const& source) override {
            for (auto& target : this->targets) {
                target->subscribeTo(source);
            }
        }

        bool can_subscribe_to(plug const& source) const override {
            for (auto& target : this->targets) {
                if (!target->can_subscribe_to(source)) return false;
            }
            return true;
        }

        std::string repr() const override {
            return "fanout";
        }
    };
}

namespace bsk {
    /*! @brief Simulation System Model Class */
    class SysModel {
    public:
        virtual ~SysModel() {}
        virtual void SelfInit() {}
        virtual void IntegratedInit() {}
        virtual void UpdateState(uint64_t CurrentSimNanos [[maybe_unused]]) {}
        virtual void Reset(uint64_t CurrentSimNanos [[maybe_unused]]) {}

        virtual inputs getInputs() { return {}; }
        virtual outputs getOutputs() const { return {}; }

        void subscribeTo(plug const& source) {
            auto inputs = this->getInputs();
            return static_cast<socket*>(&inputs)->subscribeTo(source);
        }

    public:
        std::string ModelTag = "";
        std::uint64_t CallCounts = 0;
        std::uint32_t RNGSeed = 0x1badcad1;
        std::int64_t moduleID;
    };
}
