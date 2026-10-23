#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "Publisher.hpp"
#include "Subscriber.hpp"
#include "Settings.hpp"

namespace rbq_sdk {

/**
 * ROS2 service server over bare DDS, wire-compatible with rmw_cyclonedds clients.
 *
 * Service "/rbq/estop" maps to topics "rq/rbq/estopRequest" / "rr/rbq/estopReply".
 * Req/Res IDL types must carry leading rmw_client_guid/rmw_sequence_id members
 * (the 16-byte rmw request header); Reply echoes them so the handler never does.
 *
 * The handler runs on the DDS listener thread. Call the Reply inline for a
 * synchronous response, or move it into a deferred context (e.g. a Qt queued
 * lambda) — it shares ownership of the writer, so late replies are safe.
 */
template <typename Req, typename Res>
class ServiceServer {
public:
    using Reply   = std::function<void(Res)>;
    using Handler = std::function<void(const Req&, const Reply&)>;

    ServiceServer() = default;

    ServiceServer(const std::string& service_name, Handler handler)
        : pub_(std::make_shared<Publisher<Res>>("rr" + service_name + "Reply",
                                                srv_writer_qos())),
          sub_(std::make_shared<Subscriber<Req>>(
              [pub = pub_, handler = std::move(handler)](const Req& req) {
                  Reply reply = [pub,
                                 guid = req.rmw_client_guid(),
                                 seq  = req.rmw_sequence_id()](Res res) {
                      res.rmw_client_guid(guid);
                      res.rmw_sequence_id(seq);
                      pub->write(res);
                  };
                  handler(req, reply);
              },
              "rq" + service_name + "Request", srv_reader_qos()))
    {}

private:
    std::shared_ptr<Publisher<Res>>  pub_;   ///< declared first: outlives sub_'s callbacks via shared Reply
    std::shared_ptr<Subscriber<Req>> sub_;
};

} // namespace rbq_sdk
