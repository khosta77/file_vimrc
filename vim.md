# Команды

# Конфигурация

```cmd
"============================================================================================================
" Конфигурационный файл для: vim 
" Автор: khosta77
" Дата последнего изменения: 27.07.23
"============================================================================================================


" Настройка плагинов
set nocompatible " Отключает обратную совместимость с Vi
filetype off     " Отключения обнаружения типа файла

""" Установка расширения
" Установка пути к среде выполнения так, чтобы он включал Vundle и инициализировал
set rtp+=~/.vim/bundle/Vundle.vim
call vundle#begin()

" В качестве альтернативы, укажите путь, по которому Vundle должен установить плагины
" Вызовите vundle#begin('~/some/path/here')
" пусть Vundle управляет Vundle, требуется
Plugin 'VundleVim/Vundle.vim'

" Ниже приведены примеры поддерживаемых различных форматов.
" Сохраняйте команды плагина между vundle#begin/end.
" плагин для репозитория GitHub
Plugin 'tpope/vim-fugitive'

" плагин из http://vim-scripts.org/vim/scripts.html
" Плагин 'L9'
" Плагин Git не размещен на GitHub
Plugin 'git://git.wincent.com/command-t.git'

" репозитории git на вашем локальном компьютере (т.е. при работе с вашим собственным плагином)
" Файл плагина:///home/gmarik/путь/к/плагину"
" Скрипт sparkup vim находится в подкаталоге этого репозитория под названием vim.
" Передайте путь, чтобы правильно задать путь к среде выполнения.
Plugin 'rstacruz/sparkup', {'rtp': 'vim/'}

" Установите L9 и избегайте конфликта имен, если вы уже установили
" другую версию где-то в другом месте.
" Плагин 'ascenator/L9', {'name': 'newL9'}
Plugin 'scrooloose/nerdtree'

Plugin 'flazz/vim-colorschemes'
Plugin 'tpope/vim-surround'
Plugin 'octol/vim-cpp-enhanced-highlight'
Plugin 'fatih/vim-go'

" Все ваши плагины должны быть добавлены перед следующей строкой
call vundle#end()

" Включаем авто-определение файлов с уже настроенным плагином
filetype plugin indent on

"----------------- Удалить если ни на что не повличет
" To ignore plugin indent changes, instead use:
"filetype plugin on
"
" Brief help
" :PluginList       - lists configured plugins
" :PluginInstall    - installs plugins; append `!` to update or just :PluginUpdate
" :PluginSearch foo - searches for foo; append `!` to refresh local cache
" :PluginClean      - confirms removal of unused plugins; append `!` to auto-approve removal
"
" see :h vundle for more details or wiki for FAQ
" Put your non-Plugin stuff after this line
"---------------------------------------------------------------

""" Настройка
"" табы
set expandtab          " В режиме вставки заменяет символ табуляции на соответствующее количество пробелов
set tabstop=4          " Установка размеров таба
set shiftwidth=4
set smarttab           " Табы
set softtabstop=4      " 4 пробела в табе
"set noexpandtab
"" 
set colorcolumn=110    " Включить полосу, которая справа обозначает границу
highlight ColorColumn ctermbg=darkgray
set number             " чтобы слева были строки подписаны
set autoindent         " Автоотступ
set mousehide          " Спрятать курсор мыши когда набираем текст
set mouse=a            " Включить поддержку мыши
set termencoding=utf-8 " Кодировка терминала
set novisualbell       " Не мигать 
set t_vb=              " Не пищать! (Опции 'не портить текст', к сожалению, нету)
" Удобное поведение backspace
set backspace=indent,eol,start whichwrap+=<,>,[,]
set showtabline=1      " Вырубаем черточки на табах

" Включить обозначение пробелов
"set list lcs=tab:>\ ,space:. 
"set list
set lcs+=space:·
 
""" Подсветка синтаксиса
" Подсвечиваем все что можно подсвечивать в Python
let python_highlight_all = 1

" Подсвечиваяем все что можно подсвечивать в C/C++
let g:cpp_class_scope_highlight = 1
let g:cpp_member_variable_highlight = 1
let g:cpp_class_decl_highlight = 1
let g:cpp_posix_standard = 1
let g:cpp_experimental_simple_template_highlight = 1
let g:cpp_experimental_template_highlight = 1
let g:cpp_concepts_highlight = 1
let g:cpp_no_function_highlight = 1
let c_no_curly_error=1

" https://github.com/octol/vim-cpp-enhanced-highlight
" Включаем 256 цветов в терминале
" Нужно во многих терминалах, например в gnome-terminal
set t_Co=256

" Перед сохранением вырезаем пробелы на концах (только в .py файлах)
" autocmd BufWritePre *.py normal m`:%s/\s\+$//e ``
" В .py файлах включаем умные отступы после ключевых слов
" autocmd BufRead *.py set smartindent cinwords=if,elif,else,for,while,try,except,finally,def,class

syntax on "Включить подсветку синтаксиса

" С/C++ файлы
" Расставлять отступы в стиле С
autocmd filetype c,cpp set cin

" make-файлы
" В make-файлах нам не нужно заменять табуляцию пробелами
autocmd filetype make set noexpandtab
autocmd filetype make set nocin

" html-файлы
" Не расставлять отступы в стиле С в html файлах
autocmd filetype html set noexpandtab
autocmd filetype html set nocin
autocmd filetype html set textwidth=160

" css-файлы
" Не расставлять отступы в стиле C и не заменять табуляцию пробелами
autocmd filetype css set nocin
autocmd filetype css set noexpandtab

" python-файлы
" Не расставлять отступы в стиле С
autocmd filetype python set nocin

" go-файлы
" Будем и go файлах отступы делать
autocmd filetype go set cin

" NERDTree
" Открывать дерево по нажаить Ctrl+n
map <c-b> :NERDTreeToggle<cr>

" Немного магии, если мы запустим Vim без указания файла для редактирования,
" то дерево будет открыто, а если будет указан файл, то дерево 
" открыто не будет                                                   
autocmd StdinReadPre * let s:std_in=1                                           
autocmd VimEnter * if argc() == 0 && !exists("s:std_in") | NERDTree | endif     

" Открывать новые окна справа
set splitright

" let g:clang_library_path='/usr/lib/llvm-14/lib/libclang-14.so.1'
let g:ycm_global_ycm_extra_conf = "~/.vim/.ycm_extra_conf.py"
let g:ycm_key_list_select_completion=[]
let g:ycm_key_list_previous_completion=[]

" Переносим на другую строчку, разрываем строки
set wrap
set linebreak

" Вырубаем .swp и ~ (резервные) файлы
set nobackup
set noswapfile
set encoding=utf-8 " Кодировка файлов по умолчанию
set fileencodings=utf8,cp1251

set clipboard=unnamed
set ruler

" Авто дополнение при нажатии кнопки
set hidden
nnoremap <C-N> :bnext<CR>
nnoremap <C-P> :bprev<CR>

" Выключаем звук в Vim
set visualbell t_vb=

"Переключение табов по CMD+number для MacVim
if has("gui_macvim")
  " Press Ctrl-Tab to switch between open tabs (like browser tabs) to 
  " the right side. Ctrl-Shift-Tab goes the other way.
  noremap <C-Tab> :tabnext<CR>
  noremap <C-S-Tab> :tabprev<CR>

  " Switch to specific tab numbers with Command-number
  noremap <D-1> :tabn 1<CR>
  noremap <D-2> :tabn 2<CR>
  noremap <D-3> :tabn 3<CR>
  noremap <D-4> :tabn 4<CR>
  noremap <D-5> :tabn 5<CR>
  noremap <D-6> :tabn 6<CR>
  noremap <D-7> :tabn 7<CR>
  noremap <D-8> :tabn 8<CR>
  noremap <D-9> :tabn 9<CR>
  " Command-0 goes to the last tab
  noremap <D-0> :tablast<CR>
endif

set guifont=Monaco:h18

"colorscheme ayu " бред
"colorscheme afterglow " слишком серая, в С++, но в других вроде норм
"colorscheme abstract " слишком контрастная, но прикольная
"colorscheme gruvbox " как бумага все становится
"colorscheme happy_hacking " на С++ хорошо выглядит 
"colorscheme Iceberg
"colorscheme meta5
colorscheme molokai " OK
"colorscheme tender " 50 50
" https://github.com/rafi/awesome-vim-colorschemes?ysclid=la9oa2894v712293162
```
