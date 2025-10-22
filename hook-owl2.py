# PyInstaller hook for owl2 package
from PyInstaller.utils.hooks import collect_all, collect_submodules

# 收集owl2包的所有子模块
hiddenimports = collect_submodules('owl2')

# 收集所有数据文件、二进制文件和元数据
datas, binaries, hiddenimports_tmp = collect_all('owl2')

hiddenimports += hiddenimports_tmp

