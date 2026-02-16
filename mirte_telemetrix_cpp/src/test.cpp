#include <mirte_telemetrix_cpp/mirte-board.hpp>
int main() {
  // Your code here
  Mirte_Board_pico board; //(/*tmx, s_node*/);
  Mirte_Board_pcb pcb(std::make_shared<Mirte_Board_pico>(board), "v06");
  std::shared_ptr<Mirte_Board> board = std::make_shared<Mirte_Board_pcb>(pcb);
  std::cout << board->resolvePin("GP0") << std::endl;
  auto x = board->resolveConnector("MC2-B");
  for (auto i : x) {
    std::cout << i.first << " " << i.second << std::endl;
  }
  return 0;
}
