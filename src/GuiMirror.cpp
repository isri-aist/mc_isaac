#include "GuiMirror.h"

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <cmath>
#include <functional>

namespace fs = std::filesystem;

namespace mc_isaac
{

namespace
{

/** Element key shared with the server: category path and name joined by the ASCII unit separator */
std::string widget_key(const mc_control::ElementId & id)
{
  std::string out;
  for(const auto & c : id.category) { out += c + '\x1f'; }
  return out + id.name;
}

std::vector<double> rgba(const mc_rtc::gui::Color & c)
{
  return {c.r, c.g, c.b, c.a};
}

/** Copy of a configuration in a new document (mc_rtc::Configuration copies are views on the same document) */
mc_rtc::Configuration clone(const mc_rtc::Configuration & c)
{
  mc_rtc::Configuration out;
  out.load(c);
  return out;
}

/** JSON schema helpers, same resolution as mc_rtc-imgui (mc-rtc-magnum) */
void resolve_ref(const fs::path & path,
                 mc_rtc::Configuration conf,
                 const std::function<mc_rtc::Configuration(const fs::path &)> & load)
{
  if(conf.size())
  {
    for(size_t i = 0; i < conf.size(); ++i) { resolve_ref(path, conf[i], load); }
    return;
  }
  for(const auto & k : conf.keys())
  {
    if(k == "$ref")
    {
      auto ref = load(fs::weakly_canonical(path.parent_path() / static_cast<std::string>(conf(k))));
      for(const auto & rk : ref.keys())
      {
        if(!conf.has(rk)) { conf.add(rk, ref(rk)); }
      }
      conf.remove("$ref");
    }
    else
    {
      resolve_ref(path, conf(k), load);
    }
  }
}

void resolve_all_of(mc_rtc::Configuration conf)
{
  if(conf.size())
  {
    for(size_t i = 0; i < conf.size(); ++i) { resolve_all_of(conf[i]); }
    return;
  }
  for(const auto & k : conf.keys())
  {
    if(k == "allOf")
    {
      std::vector<mc_rtc::Configuration> all_of = conf("allOf");
      for(auto & c : all_of)
      {
        resolve_all_of(c);
        conf.load(c);
      }
      conf.remove("allOf");
    }
    else
    {
      resolve_all_of(conf(k));
    }
  }
}

} // namespace

void GuiMirror::started()
{
  Markers::started();
  root_ = Node{};
  elements_.clear();
  tables_.clear();
  formStack_.clear();
  seenPlots_.clear();
}

void GuiMirror::stopped()
{
  Markers::stopped();
  formStack_.clear();
  activePlots_ = seenPlots_;
}

GuiMirror::Item & GuiMirror::item(const ElementId & id, const char * type)
{
  Node * node = &root_;
  for(const auto & name : id.category)
  {
    auto it = std::find_if(node->children.begin(), node->children.end(),
                           [&](const auto & child) { return child->name == name; });
    if(it == node->children.end())
    {
      node->children.push_back(std::make_unique<Node>());
      node->children.back()->name = name;
      node = node->children.back().get();
    }
    else
    {
      node = it->get();
    }
  }
  const auto key = widget_key(id);
  elements_[key] = id;
  auto & out = node->items.emplace_back();
  out.desc.add("k", key);
  out.desc.add("n", id.name);
  out.desc.add("t", type);
  out.desc.add("s", id.sid);
  if(std::string(type) == "Table") { tables_[key] = {node, node->items.size() - 1}; }
  return out;
}

void GuiMirror::label(const ElementId & id, const std::string & txt)
{
  item(id, "Label").desc.add("v", txt);
}

void GuiMirror::array_label(const ElementId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data)
{
  auto & d = item(id, "ArrayLabel").desc;
  d.add("l", labels);
  d.add("v", data);
}

void GuiMirror::button(const ElementId & id)
{
  item(id, "Button");
}

void GuiMirror::checkbox(const ElementId & id, bool state)
{
  item(id, "Checkbox").desc.add("v", state);
}

void GuiMirror::string_input(const ElementId & id, const std::string & data)
{
  item(id, "StringInput").desc.add("v", data);
}

void GuiMirror::integer_input(const ElementId & id, int data)
{
  item(id, "IntegerInput").desc.add("v", data);
}

void GuiMirror::number_input(const ElementId & id, double data)
{
  item(id, "NumberInput").desc.add("v", data);
}

void GuiMirror::number_slider(const ElementId & id, double data, double min, double max)
{
  auto & d = item(id, "NumberSlider").desc;
  d.add("v", data);
  d.add("min", min);
  d.add("max", max);
}

void GuiMirror::array_input(const ElementId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data)
{
  auto & d = item(id, "ArrayInput").desc;
  d.add("l", labels);
  d.add("v", data);
}

void GuiMirror::combo_input(const ElementId & id, const std::vector<std::string> & values, const std::string & data)
{
  auto & d = item(id, "ComboInput").desc;
  d.add("o", values);
  d.add("v", data);
}

void GuiMirror::data_combo_input(const ElementId & id, const std::vector<std::string> & ref, const std::string & data)
{
  // the values are resolved by the server from the GUI data store (sent alongside)
  auto & d = item(id, "DataComboInput").desc;
  d.add("r", ref);
  d.add("v", data);
}

void GuiMirror::table_start(const ElementId & id, const std::vector<std::string> & header)
{
  auto & it = item(id, "Table");
  it.desc.add("h", header);
  it.table = true;
}

void GuiMirror::table_row(const ElementId & id, const std::vector<std::string> & data)
{
  auto it = tables_.find(widget_key(id));
  if(it != tables_.end()) { it->second.first->items[it->second.second].rows.push_back(data); }
}

void GuiMirror::schema(const ElementId & id, const std::string & schema)
{
  loadSchemaDir(schema);
  item(id, "Schema").desc.add("schema", schema);
}

void GuiMirror::form(const ElementId & id)
{
  auto & it = item(id, "Form");
  it.form = std::make_unique<FormNode>();
  formStack_ = {it.form.get()};
}

mc_rtc::Configuration & GuiMirror::field(const char * kind, const std::string & name, bool required)
{
  if(formStack_.empty())
  {
    orphanField_ = mc_rtc::Configuration{};
    return orphanField_;
  }
  auto & parent = *formStack_.back();
  auto & node = *parent.fields.emplace_back(std::make_unique<FormNode>());
  node.desc.add("kind", kind);
  node.desc.add("name", name);
  node.desc.add("required", required);
  return node.desc;
}

void GuiMirror::pushField(const char * kind, const std::string & name, bool required)
{
  field(kind, name, required);
  if(!formStack_.empty()) { formStack_.push_back(formStack_.back()->fields.back().get()); }
}

void GuiMirror::form_checkbox(const ElementId &, const std::string & name, bool required, bool def, bool user_def)
{
  auto & d = field("checkbox", name, required);
  d.add("default", def);
  d.add("user_default", user_def);
}

void GuiMirror::form_integer_input(const ElementId &, const std::string & name, bool required, int def, bool user_def)
{
  auto & d = field("integer", name, required);
  d.add("default", def);
  d.add("user_default", user_def);
}

void GuiMirror::form_number_input(const ElementId &,
                                  const std::string & name,
                                  bool required,
                                  double def,
                                  bool user_def)
{
  auto & d = field("number", name, required);
  d.add("default", def);
  d.add("user_default", user_def);
}

void GuiMirror::form_string_input(const ElementId &,
                                  const std::string & name,
                                  bool required,
                                  const std::string & def,
                                  bool user_def)
{
  auto & d = field("string", name, required);
  d.add("default", def);
  d.add("user_default", user_def);
}

void GuiMirror::form_array_input(const ElementId &,
                                 const std::string & name,
                                 bool required,
                                 const std::vector<std::string> & labels,
                                 const Eigen::VectorXd & def,
                                 bool fixed_size,
                                 bool user_def)
{
  auto & d = field("array", name, required);
  d.add("labels", labels);
  d.add("default", def);
  d.add("fixed", fixed_size);
  d.add("user_default", user_def);
}

void GuiMirror::form_combo_input(const ElementId &,
                                 const std::string & name,
                                 bool required,
                                 const std::vector<std::string> & values,
                                 bool send_index,
                                 int def)
{
  auto & d = field("combo", name, required);
  d.add("values", values);
  d.add("send_index", send_index);
  d.add("default_index", def);
}

void GuiMirror::form_data_combo_input(const ElementId &,
                                      const std::string & name,
                                      bool required,
                                      const std::vector<std::string> & ref,
                                      bool send_index)
{
  auto & d = field("data_combo", name, required);
  d.add("ref", ref);
  d.add("send_index", send_index);
}

void GuiMirror::form_point3d_input(const ElementId &,
                                   const std::string & name,
                                   bool required,
                                   const Eigen::Vector3d & def,
                                   bool user_def,
                                   bool interactive)
{
  auto & d = field("point3d", name, required);
  d.add("default", def);
  d.add("user_default", user_def);
  d.add("interactive", interactive);
}

void GuiMirror::form_rotation_input(const ElementId &,
                                    const std::string & name,
                                    bool required,
                                    const sva::PTransformd & def,
                                    bool user_def,
                                    bool interactive)
{
  // same quaternion convention as the request data (mc_rtc-imgui sends Quaterniond(rotation()))
  auto & d = field("rotation", name, required);
  d.add("default", Eigen::Quaterniond(def.rotation()));
  d.add("user_default", user_def);
  d.add("interactive", interactive);
}

void GuiMirror::form_transform_input(const ElementId &,
                                     const std::string & name,
                                     bool required,
                                     const sva::PTransformd & def,
                                     bool user_def,
                                     bool interactive)
{
  auto & d = field("transform", name, required);
  d.add("default_translation", Eigen::Vector3d(def.translation()));
  d.add("default", Eigen::Quaterniond(def.rotation()));
  d.add("user_default", user_def);
  d.add("interactive", interactive);
}

void GuiMirror::start_form_object_input(const std::string & name, bool required)
{
  pushField("object", name, required);
}

void GuiMirror::end_form_object_input()
{
  if(formStack_.size() > 1) { formStack_.pop_back(); }
}

void GuiMirror::start_form_generic_array_input(const std::string & name,
                                               bool required,
                                               std::optional<std::vector<mc_rtc::Configuration>> data)
{
  // the first nested field describes the array items
  auto & d = field("generic_array", name, required);
  if(data)
  {
    auto array = d.array("data");
    for(const auto & c : *data) { array.push(c); }
  }
  if(!formStack_.empty()) { formStack_.push_back(formStack_.back()->fields.back().get()); }
}

void GuiMirror::end_form_generic_array_input()
{
  if(formStack_.size() > 1) { formStack_.pop_back(); }
}

void GuiMirror::start_form_one_of_input(const std::string & name,
                                        bool required,
                                        const std::optional<std::pair<size_t, mc_rtc::Configuration>> & data)
{
  // each nested field is one option, the request data is [option index, option value]
  auto & d = field("one_of", name, required);
  if(data)
  {
    d.add("data_index", data->first);
    d.add("data_value", data->second);
  }
  if(!formStack_.empty()) { formStack_.push_back(formStack_.back()->fields.back().get()); }
}

void GuiMirror::end_form_one_of_input()
{
  if(formStack_.size() > 1) { formStack_.pop_back(); }
}

void GuiMirror::start_plot(uint64_t id, const std::string & title)
{
  seenPlots_.insert(id);
  auto & plot = plots_[id];
  if(plot.title != title) { plot = Plot{}; }
  plot.title = title;
}

void GuiMirror::axis(uint64_t id, const char * which, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  mc_rtc::Configuration a;
  a.add("label", legend);
  // infinite bounds: computed from the data by the server
  if(std::isfinite(range.min)) { a.add("min", range.min); }
  if(std::isfinite(range.max)) { a.add("max", range.max); }
  plots_[id].axes.add(which, a);
}

void GuiMirror::plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  axis(id, "x", legend, range);
}

void GuiMirror::plot_setup_yaxis_left(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  axis(id, "y", legend, range);
}

void GuiMirror::plot_setup_yaxis_right(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  axis(id, "y2", legend, range);
}

void GuiMirror::plot_point(uint64_t id,
                           uint64_t did,
                           const std::string & legend,
                           double x,
                           double y,
                           mc_rtc::gui::Color color,
                           mc_rtc::gui::plot::Style style,
                           mc_rtc::gui::plot::Side side)
{
  auto & plot = plots_[id];
  auto & series = plot.series[did];
  series = mc_rtc::Configuration{};
  series.add("legend", legend);
  series.add("color", rgba(color));
  series.add("style", static_cast<int>(style));
  series.add("side", static_cast<int>(side));
  plot.points.insert(plot.points.end(), {static_cast<double>(did), x, y});
}

void GuiMirror::plot_polygon(uint64_t id,
                             uint64_t did,
                             const std::string & legend,
                             const mc_rtc::gui::plot::PolygonDescription & polygon,
                             mc_rtc::gui::plot::Side side)
{
  plot_polygons(id, did, legend, {polygon}, side);
}

void GuiMirror::plot_polygons(uint64_t id,
                              uint64_t did,
                              const std::string & legend,
                              const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
                              mc_rtc::gui::plot::Side side)
{
  mc_rtc::Configuration out;
  out.add("legend", legend);
  out.add("side", static_cast<int>(side));
  auto array = out.array("polygons");
  for(const auto & p : polygons)
  {
    auto poly = array.object();
    std::vector<double> points;
    for(const auto & pt : p.points()) { points.insert(points.end(), {pt[0], pt[1]}); }
    poly.add("points", points);
    poly.add("outline", rgba(p.outline()));
    poly.add("fill", rgba(p.fill()));
    poly.add("closed", p.closed());
  }
  plots_[id].polygons[did] = out;
}

mc_rtc::Configuration GuiMirror::toConfig(const FormNode & node)
{
  auto out = clone(node.desc);
  if(!node.fields.empty())
  {
    auto fields = out.array("f");
    for(const auto & f : node.fields) { fields.push(toConfig(*f)); }
  }
  return out;
}

mc_rtc::Configuration GuiMirror::toConfig(const Node & node)
{
  mc_rtc::Configuration out;
  out.add("n", node.name);
  {
    auto widgets = out.array("w");
    for(const auto & it : node.items)
    {
      auto desc = clone(it.desc);
      if(it.form)
      {
        auto fields = desc.array("f");
        for(const auto & f : it.form->fields) { fields.push(toConfig(*f)); }
      }
      if(it.table)
      {
        auto rows = desc.array("rows");
        for(const auto & row : it.rows)
        {
          auto r = rows.array();
          for(const auto & cell : row) { r.push(cell); }
        }
      }
      widgets.push(desc);
    }
  }
  auto children = out.array("c");
  for(const auto & c : node.children) { children.push(toConfig(*c)); }
  return out;
}

void GuiMirror::loadSchemaDir(const std::string & dir)
{
  if(schemas_.count(dir)) { return; }
  auto & out = schemas_[dir];
  const fs::path path = fs::path(mc_rtc::JSON_SCHEMA_PATH) / dir;
  if(!fs::is_directory(path))
  {
    mc_rtc::log::error("[mc_isaac] No mc_rtc schema directory {}", path.string());
    return;
  }
  for(const auto & entry : fs::directory_iterator(path))
  {
    try
    {
      auto schema = loadSchema(fs::weakly_canonical(entry.path()));
      out.add(schema("title", entry.path().stem().string()), schema);
    }
    catch(const std::exception & e)
    {
      mc_rtc::log::error("[mc_isaac] Cannot load the schema {}: {}", entry.path().string(), e.what());
    }
  }
}

mc_rtc::Configuration GuiMirror::loadSchema(const fs::path & path)
{
  auto it = schemaFiles_.find(path.string());
  if(it != schemaFiles_.end()) { return it->second; }
  // cached before resolving so that recursive references terminate
  auto & schema = schemaFiles_[path.string()];
  schema.load(path.string());
  resolve_ref(path, schema, [this](const fs::path & p) { return loadSchema(p); });
  resolve_all_of(schema);
  return schema;
}

std::optional<mc_rtc::Configuration> GuiMirror::collect()
{
  mc_rtc::Configuration out;
  bool changed = full_;
  const auto tree = toConfig(root_);
  auto dump = tree.dump();
  if(full_ || dump != lastTree_)
  {
    out.add("tree", tree);
    lastTree_ = std::move(dump);
    changed = true;
  }
  dump = data_.dump();
  if(full_ || dump != lastData_)
  {
    out.add("data", data_);
    lastData_ = std::move(dump);
    changed = true;
  }
  if(full_) { sentSchemas_.clear(); }
  {
    mc_rtc::Configuration schemas;
    bool any = false;
    for(const auto & [dir, s] : schemas_)
    {
      if(sentSchemas_.insert(dir).second)
      {
        schemas.add(dir, s);
        any = true;
      }
    }
    if(any)
    {
      out.add("schemas", schemas);
      changed = true;
    }
  }
  if(full_ || activePlots_ != sentActivePlots_)
  {
    out.add("plots_active", std::vector<uint64_t>(activePlots_.begin(), activePlots_.end()));
    sentActivePlots_ = activePlots_;
    changed = true;
  }
  {
    auto plots = out.array("plots");
    for(auto it = plots_.begin(); it != plots_.end();)
    {
      if(!activePlots_.count(it->first))
      {
        it = plots_.erase(it);
        continue;
      }
      auto & p = it->second;
      if(!p.points.empty() || !p.polygons.empty() || full_)
      {
        auto plot = plots.object();
        plot.add("id", it->first);
        plot.add("title", p.title);
        plot.add("axes", p.axes);
        auto series = plot.array("series");
        for(const auto & [did, s] : p.series)
        {
          auto entry = clone(s);
          entry.add("did", did);
          series.push(entry);
        }
        plot.add("points", p.points);
        auto polygons = plot.array("polygons");
        for(const auto & [did, poly] : p.polygons)
        {
          auto entry = clone(poly);
          entry.add("did", did);
          polygons.push(entry);
        }
        p.points.clear();
        changed = true;
      }
      ++it;
    }
  }
  out.add("full", full_);
  full_ = false;
  if(!changed) { return std::nullopt; }
  return out;
}

void GuiMirror::widgetRequest(const std::string & key, const mc_rtc::Configuration * data)
{
  auto it = elements_.find(key);
  if(it == elements_.end())
  {
    mc_rtc::log::warning("[mc_isaac] GUI element {} is gone, request ignored", key);
    return;
  }
  if(data) { send_request(it->second, *data); }
  else { send_request(it->second); }
}

} // namespace mc_isaac
