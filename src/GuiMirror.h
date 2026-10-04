#pragma once

#include "Markers.h"

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace mc_isaac
{

/** In-process mc_rtc GUI client mirroring the whole 2D GUI in the Isaac window (plugin option gui: full).
 *
 * The 3D elements are still handled by Markers. Each parse of the GUI state records the 2D elements in a category
 * tree; collect() serializes it for the server as {n: name, w: [widgets], c: [sub-categories]} where a widget is
 * {k: key, n: name, t: type, s: stack id, ...type data} (see the handlers below). Forms keep their nested fields
 * (f), schemas are sent once, resolved from the mc_rtc JSON schema folder, and plot points are accumulated between
 * two sends. The server sends the user's actions back by element key (widgetRequest()).
 */
struct GuiMirror : public Markers
{
  /** Payload for the server {tree, data, schemas, plots, plots_active, full}, nullopt when nothing changed */
  std::optional<mc_rtc::Configuration> collect();

  /** The server (re)created its mirror: the next collect() sends everything */
  void resync() { full_ = true; }

  /** User action from the Isaac GUI on the element key: request with data, or without (buttons, checkboxes) */
  void widgetRequest(const std::string & key, const mc_rtc::Configuration * data);

protected:
  void started() override;
  void stopped() override;

  void label(const ElementId & id, const std::string & txt) override;
  void array_label(const ElementId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data) override;
  void button(const ElementId & id) override;
  void checkbox(const ElementId & id, bool state) override;
  void string_input(const ElementId & id, const std::string & data) override;
  void integer_input(const ElementId & id, int data) override;
  void number_input(const ElementId & id, double data) override;
  void number_slider(const ElementId & id, double data, double min, double max) override;
  void array_input(const ElementId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data) override;
  void combo_input(const ElementId & id, const std::vector<std::string> & values, const std::string & data) override;
  void data_combo_input(const ElementId & id, const std::vector<std::string> & ref, const std::string & data) override;
  void table_start(const ElementId & id, const std::vector<std::string> & header) override;
  void table_row(const ElementId & id, const std::vector<std::string> & data) override;
  void schema(const ElementId & id, const std::string & schema) override;

  void form(const ElementId & id) override;
  void form_checkbox(const ElementId &, const std::string & name, bool required, bool def, bool user_def) override;
  void form_integer_input(const ElementId &, const std::string & name, bool required, int def, bool user_def) override;
  void form_number_input(const ElementId &, const std::string & name, bool required, double def, bool user_def) override;
  void form_string_input(const ElementId &,
                         const std::string & name,
                         bool required,
                         const std::string & def,
                         bool user_def) override;
  void form_array_input(const ElementId &,
                        const std::string & name,
                        bool required,
                        const std::vector<std::string> & labels,
                        const Eigen::VectorXd & def,
                        bool fixed_size,
                        bool user_def) override;
  void form_combo_input(const ElementId &,
                        const std::string & name,
                        bool required,
                        const std::vector<std::string> & values,
                        bool send_index,
                        int def) override;
  void form_data_combo_input(const ElementId &,
                             const std::string & name,
                             bool required,
                             const std::vector<std::string> & ref,
                             bool send_index) override;
  void form_point3d_input(const ElementId &,
                          const std::string & name,
                          bool required,
                          const Eigen::Vector3d & def,
                          bool user_def,
                          bool interactive) override;
  void form_rotation_input(const ElementId &,
                           const std::string & name,
                           bool required,
                           const sva::PTransformd & def,
                           bool user_def,
                           bool interactive) override;
  void form_transform_input(const ElementId &,
                            const std::string & name,
                            bool required,
                            const sva::PTransformd & def,
                            bool user_def,
                            bool interactive) override;
  void start_form_object_input(const std::string & name, bool required) override;
  void end_form_object_input() override;
  void start_form_generic_array_input(const std::string & name,
                                      bool required,
                                      std::optional<std::vector<mc_rtc::Configuration>> data) override;
  void end_form_generic_array_input() override;
  void start_form_one_of_input(const std::string & name,
                               bool required,
                               const std::optional<std::pair<size_t, mc_rtc::Configuration>> & data) override;
  void end_form_one_of_input() override;

  void start_plot(uint64_t id, const std::string & title) override;
  void plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range) override;
  void plot_setup_yaxis_left(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range) override;
  void plot_setup_yaxis_right(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range) override;
  void plot_point(uint64_t id,
                  uint64_t did,
                  const std::string & legend,
                  double x,
                  double y,
                  mc_rtc::gui::Color color,
                  mc_rtc::gui::plot::Style style,
                  mc_rtc::gui::plot::Side side) override;
  void plot_polygon(uint64_t id,
                    uint64_t did,
                    const std::string & legend,
                    const mc_rtc::gui::plot::PolygonDescription & polygon,
                    mc_rtc::gui::plot::Side side) override;
  void plot_polygons(uint64_t id,
                     uint64_t did,
                     const std::string & legend,
                     const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
                     mc_rtc::gui::plot::Side side) override;

private:
  /** Form field: description + nested fields (object, generic array template, one-of options) */
  struct FormNode
  {
    mc_rtc::Configuration desc;
    std::vector<std::unique_ptr<FormNode>> fields;
  };

  struct Item
  {
    mc_rtc::Configuration desc;
    std::unique_ptr<FormNode> form;
    std::vector<std::vector<std::string>> rows;
    bool table = false;
  };

  struct Node
  {
    std::string name;
    std::vector<Item> items;
    std::vector<std::unique_ptr<Node>> children;
  };

  struct Plot
  {
    std::string title;
    mc_rtc::Configuration axes;
    /** did -> {legend, color, style, side} */
    std::map<uint64_t, mc_rtc::Configuration> series;
    /** did x y triplets received since the last send */
    std::vector<double> points;
    /** did -> latest polygons {legend, side, polygons: [{points, outline, fill, closed}]} */
    std::map<uint64_t, mc_rtc::Configuration> polygons;
  };

  /** New element {k, n, t, s} in its category node (created on the way), registered for requests */
  Item & item(const ElementId & id, const char * type);
  /** New field {kind, name, required} in the current form container (a dummy outside of a form) */
  mc_rtc::Configuration & field(const char * kind, const std::string & name, bool required);
  /** New container field (object): the next fields go inside it until the matching end_* call */
  void pushField(const char * kind, const std::string & name, bool required);
  /** Serialized category {n, w, c} / form field {..., f} for the server */
  static mc_rtc::Configuration toConfig(const Node & node);
  static mc_rtc::Configuration toConfig(const FormNode & node);
  /** Load the schemas of an mc_rtc JSON schema directory ({title: schema}), once */
  void loadSchemaDir(const std::string & dir);
  /** JSON schema file with $ref and allOf resolved (cached by path) */
  mc_rtc::Configuration loadSchema(const std::filesystem::path & path);
  /** Axis {label, min?, max?} of a plot (infinite bounds omitted) */
  void axis(uint64_t id, const char * which, const std::string & legend, const mc_rtc::gui::plot::Range & range);

  Node root_;
  /** element key -> mc_rtc element of the last parse */
  std::map<std::string, ElementId> elements_;
  /** table key -> (category node, item index) of the last parse */
  std::map<std::string, std::pair<Node *, size_t>> tables_;
  /** current form and its nested containers */
  std::vector<FormNode *> formStack_;
  /** dummy field target for form elements outside of a form */
  mc_rtc::Configuration orphanField_;

  /** schema directory -> {title: resolved schema}; sent once (again after resync()) */
  std::map<std::string, mc_rtc::Configuration> schemas_;
  std::set<std::string> sentSchemas_;
  std::map<std::string, mc_rtc::Configuration> schemaFiles_;

  std::map<uint64_t, Plot> plots_;
  std::set<uint64_t> activePlots_, seenPlots_, sentActivePlots_;

  std::string lastTree_, lastData_;
  bool full_ = true;
};

} // namespace mc_isaac
