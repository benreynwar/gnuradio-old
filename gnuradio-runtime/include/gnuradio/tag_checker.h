#include <vector>

#include <gnuradio/tags.h>

namespace gr {

    class tag_checker
    {
    public:
      tag_checker(std::vector<tag_t> &tags) {
        d_tags = tags;
        if (tags.size() > 0) {
          d_has_next_tag = true;
          d_next_tag_index = 0;
          d_next_tag = tags[0];
        }
      };
      ~tag_checker() {};
      void get_tags(std::vector<tag_t> &tag_list, unsigned int offset) {
        while (d_has_next_tag && (offset >= d_next_tag.offset)) {
          if (offset == d_next_tag.offset) {
            tag_list.push_back(d_next_tag);
          }
          d_next_tag_index += 1;
          if (d_next_tag_index >= d_tags.size()) {
            d_has_next_tag = false;
          } else {
            d_next_tag = d_tags[d_next_tag_index];
          }
        }
      };
    private:
      std::vector<tag_t> d_tags;
      tag_t d_next_tag;
      unsigned int d_next_tag_index;
      bool d_has_next_tag;
    };
      
}
