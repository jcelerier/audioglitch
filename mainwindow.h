#pragma once
#include <QMainWindow>
#include <QImage>
#include <QFormLayout>
#include <QLabel>

#include <vector>
#include <array>
#include <faust/dsp/llvm-dsp.h>

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

  void updateImage();

private:
  QImage m_sourceImage;
  QLabel* m_image{};
  llvm_dsp_factory* m_factory{};
  llvm_dsp* m_dsp[4]{};

  std::vector<std::array<float*, 4>> m_guis;
  QFormLayout* m_controls{};
};
