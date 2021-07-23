-- Functions

local query_syntax_stack = function()
  local _, line, col, _ = unpack(vim.fn.getpos('.'))
  for _, id in ipairs(vim.fn.synstack(line, col)) do
    print(vim.fn.synIDattr(id, 'name'))
  end
end

local tab_open_term = function(cmd)
  local cmd_name = cmd or os.getenv('SHELL')
  local cur_bufs = vim.api.nvim_list_bufs()
  local shell_buf = vim.tbl_filter(function(buf)
    return vim.api.nvim_buf_get_option(buf, 'buftype') == 'terminal'
      and vim.api.nvim_buf_get_name(buf):find(cmd_name)
  end, cur_bufs)
  if #shell_buf == 0 then
    vim.cmd('tabnew')
    vim.g.shell_tab_num = vim.api.nvim_get_current_tabpage()
    if cmd then
      vim.cmd('terminal ' .. cmd_name)
    else
      vim.cmd('terminal')
    end
    vim.g.shell_channel_id = vim.bo.channel
    vim.bo.buflisted = false
    vim.cmd('startinsert')
  else
    vim.cmd(vim.g.shell_tab_num .. 'tabnext')
    if cmd then
      vim.cmd('startinsert')
    end
  end
end

-- echo different formats and the corresponding char for a given number
local echo_formats_and_char = function()
  local sel = table.concat(require('xx.utils').fetch_selection('v'), '')
  local hex_regex = vim.regex [[\v\c^(\\x|0x|\\u|u\+)]]
  local dec_regex = vim.regex [[\v^\d+$]]
  local output
  if dec_regex:match_str(sel) then
    output = '0x' .. string.format('%x', sel)
  else -- hexdecimal
    local m_beg, m_end = hex_regex:match_str(sel)
    if m_beg then
      sel = '0x' .. sel:sub(m_end + 1)
      print(sel)
    else
      sel = '0x' .. sel
    end
    output = string.format('%d', sel)
  end
  local symbol = vim.fn.nr2char(output)
  vim.fn.setreg('"', symbol)
  print('<' .. sel .. '> ' .. output .. ' ' .. symbol)
end

local success, wk = pcall(require, 'which-key')
if not success then
  return
end

wk.register({
  q = {
    name = '+quit/close',
    b = { '<cmd>silent! bdelete!<cr>', 'Close buffer' },
    w = { '<cmd>silent! bwipeout!<cr>', 'Wipeout buffer' },
    t = { '<cmd>tabclose<cr>', 'Close tab' },
    d = { '<cmd>let b:coc_diagnostic_disable = 1<Bar>edit<cr>', 'Close diagnostic' },
    c = { '<cmd>cclose<cr>', 'Close quickfix' },
    l = { '<cmd>lclose<cr>', 'Close location list' },
  },
  c = {
    name = '+change/command',
    d = { '<cmd>lcd %:p:h<cr>', 'Change window directory' },
    i = { '<cmd>IndentBlanklineToggle<cr>', 'Toggle indent line' },
  },
  s = {
    h = { vim.fn.SyntaxAttr, 'Syntax highlighting group' },
    H = { query_syntax_stack, 'Syntax highlighting stack' },
    t = { '<cmd>TSHighlightCapturesUnderCursor<cr>', 'TreeSitter highlighting under cursor' },
    T = { '<cmd>terminal tokei<cr>', 'Tokei code statistic' },
    f = { 'ga', 'Show different formats of character' },
  },
  n = {
    name = '+new',
    t = { '<cmd>tabnew %<cr>', 'New tabpage' },
    x = { ':read !figlet<space>', 'Insert new figlet symbol' },
  },
  ['<C-t>'] = { function() tab_open_term('vit') end, 'Open vit in new tab' },
  x = { function() tab_open_term() end, 'Open terminal in new tab' },
  yp = { "<cmd>let @+=expand('%:p')<cr>", 'Yank file path' },
}, { prefix = '<leader>' })

wk.register({
  s = {
    name = '+show',
    f = { echo_formats_and_char, 'Show different formats of character' }
  }
}, { mode = 'x', prefix = '<leader>' })
