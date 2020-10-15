/*
 * Copyright (c) 2000-2019 Stephen Williams (steve@icarus.com)
 *
 *    This source code is free software; you can redistribute it
 *    and/or modify it in source code form under the terms of the GNU
 *    General Public License as published by the Free Software
 *    Foundation; either version 2 of the License, or (at your option)
 *    any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

# include "config.h"

# include  <iostream>
# include  <set>
# include  <cstdlib>
#include   <cstring>
/*
 * This source file contains all the implementations of the Design
 * class declared in netlist.h.
 */

# include  "netlist.h"
# include  "util.h"
# include  "compiler.h"
# include  "netmisc.h"
# include  "PExpr.h"
# include  "PTask.h"
# include  <sstream>
# include  "ivl_assert.h"


#pragma GCC push_options
#pragma GCC optimize ("O0")



Design:: Design()
    : errors(0), nodes_(0), procs_(0), aprocs_(0)
{
      branches_ = 0;
      procs_idx_ = 0;
      des_precision_ = 0;
      nodes_functor_cur_ = 0;
      nodes_functor_nxt_ = 0;
      des_delay_sel_ = Design::TYP;
}

Design::~Design()
{
}

void Design::set_precision(int val)
{
      if (val < des_precision_)
	    des_precision_ = val;
}

int Design::get_precision() const
{
      return des_precision_;
}

void Design::set_delay_sel(delay_sel_t sel)
{
      des_delay_sel_ = sel;
}

const char* Design::get_delay_sel() const
{
      switch (des_delay_sel_) {
	case Design::MIN:
	    return "MINIMUM";
	    break;
	case Design::TYP:
	    return "TYPICAL";
	    break;
	case Design::MAX:
	    return "MAXIMUM";
	    break;
	default:
	    assert(0);
	    return "TYPICAL";
      }
}


uint64_t Design::scale_to_precision(uint64_t val,
				    const NetScope*scope) const
{
      int units = scope->time_unit();
      assert( units >= des_precision_ );

      while (units > des_precision_) {
	    units -= 1;
	    val *= 10;
      }

      return val;
}

NetScope* Design::make_root_scope(perm_string root, NetScope*unit_scope,
				  bool program_block, bool is_interface)
{
      NetScope *root_scope_;
      root_scope_ = new NetScope(0, hname_t(root), NetScope::MODULE, unit_scope,
				 false, program_block, is_interface);
	/* This relies on the fact that the basename return value is
	   permallocated. */
      root_scope_->set_module_name(root_scope_->basename());
      root_scopes_.push_back(root_scope_);
      return root_scope_;
}

NetScope* Design::find_root_scope()
{
      assert(root_scopes_.front());
      return root_scopes_.front();
}

list<NetScope*> Design::find_root_scopes() const
{
      return root_scopes_;
}

NetScope* Design::make_package_scope(perm_string name, NetScope*unit_scope,
				     bool is_unit)
{
      NetScope*scope;

      scope = new NetScope(0, hname_t(name), NetScope::PACKAGE, unit_scope,
			   false, false, false, is_unit);
      scope->set_module_name(scope->basename());
      packages_[name] = scope;
      return scope;
}

NetScope* Design::find_package(perm_string name) const
{
      map<perm_string,NetScope*>::const_iterator cur = packages_.find(name);
      if (cur == packages_.end())
	    return 0;

      return cur->second;
}

list<NetScope*> Design::find_package_scopes() const
{
      list<NetScope*>res;
      for (map<perm_string,NetScope*>::const_iterator cur = packages_.begin()
		 ; cur != packages_.end() ; ++cur) {
	    res.push_back (cur->second);
      }

      return res;
}

/*
 * This method locates a scope in the design, given its rooted
 * hierarchical name. Each component of the key is used to scan one
 * more step down the tree until the name runs out or the search
 * fails.
 */
NetScope* Design::find_scope(const std::list<hname_t>&path) const
{
      if (path.empty())
	    return 0;

      for (list<NetScope*>::const_iterator scope = root_scopes_.begin()
		 ; scope != root_scopes_.end(); ++ scope ) {

	    NetScope*cur = *scope;
	    if (path.front() != cur->fullname())
		  continue;

	    std::list<hname_t> tmp = path;
	    tmp.pop_front();

	    while (cur) {
		  if (tmp.empty()) return cur;

		  cur = cur->child( tmp.front() );

		  tmp.pop_front();
	    }
      }

      return 0;
}

/*
 * This method locates a scope in the design, given its rooted
 * hierarchical name. Each component of the key is used to scan one
 * more step down the tree until the name runs out or the search
 * fails.
 */
NetScope* Design::find_scope(const hname_t&path) const
{
      for (list<NetScope*>::const_iterator scope = root_scopes_.begin()
		 ; scope != root_scopes_.end(); ++ scope ) {

	    NetScope*cur = *scope;
	    if (path.peek_name() == cur->basename())
		  return cur;

      }

      return 0;
}

static bool is_design_unit(NetScope*scope)
{
      return (scope->type() == NetScope::MODULE && !scope->nested_module())
	  || (scope->type() == NetScope::PACKAGE);
}

static bool is_subroutine(NetScope::TYPE type)
{
      return type == NetScope::TASK || type == NetScope::FUNC;
}

/*
 * This method locates a scope within another scope, given its relative
 * hierarchical name. Each component of the key is used to scan one
 * more step down the tree until the name runs out or the search
 * fails.
 */
NetScope* Design::find_scope_(NetScope*scope, const std::list<hname_t>&path,
                              NetScope::TYPE type) const
{
      std::list<hname_t> tmp = path;

      do {
	    hname_t key = tmp.front();
	      /* If we are looking for a module or we are not
	       * looking at the last path component check for
	       * a name match (second line). */
	    if (scope->type() == NetScope::MODULE
		&& (type == NetScope::MODULE || tmp.size() > 1)
		&& scope->module_name()==key.peek_name()) {

		    /* Up references may match module name */

	    } else {
		  NetScope*found_scope = scope->child(key);
		  if (found_scope == 0) {
			found_scope = scope->find_import(this, key.peek_name());
			if (found_scope)
			      found_scope = found_scope->child(key);
		  }
		  scope = found_scope;
		  if (scope == 0) break;
	    }
	    tmp.pop_front();
      } while (! tmp.empty());

      return scope;
}

/*
 * This is a relative lookup of a scope by name. The starting point is
 * the scope parameter within which I start looking for the scope. If
 * I do not find the scope within the passed scope, start looking in
 * parent scopes until I find it, or I run out of parent scopes.
 */
NetScope* Design::find_scope(NetScope*scope, const std::list<hname_t>&path,
                             NetScope::TYPE type) const
{
      assert(scope);
      if (path.empty())
	    return scope;

	// Record the compilation unit scope for use later.
      NetScope*unit_scope = scope->unit();

	// First search upwards through the hierarchy.
      while (scope) {
	    NetScope*found_scope = find_scope_(scope, path, type);
	    if (found_scope)
		  return found_scope;

	      // Avoid searching the unit scope twice.
	    if (scope == unit_scope)
		  unit_scope = 0;

	      // Special case - see IEEE 1800-2012 section 23.8.1.
	    if (unit_scope && is_design_unit(scope) && is_subroutine(type)) {
		  found_scope = find_scope_(unit_scope, path, type);
		  if (found_scope)
			  return found_scope;

		  unit_scope = 0;
	    }

	    scope = scope->parent();
      }

	// If we haven't already done so, search the compilation unit scope.
      if (unit_scope) {
	    NetScope*found_scope = find_scope_(unit_scope, path, type);
	    if (found_scope)
		  return found_scope;
      }

	// Last chance. Look for the name starting at the root.
      return find_scope(path);
}

/*
 * This method locates a scope within another scope, given its relative
 * hierarchical name. Each component of the key is used to scan one
 * more step down the tree until the name runs out or the search
 * fails.
 */
NetScope* Design::find_scope_(NetScope*scope, const hname_t&path,
                              NetScope::TYPE type) const
{
	/* If we are looking for a module or we are not
	 * looking at the last path component check for
	 * a name match (second line). */
      if ((scope->type() == NetScope::MODULE) && (type == NetScope::MODULE)
	  && (scope->module_name() == path.peek_name())) {
	      /* Up references may match module name */
	    return scope;
      }
      NetScope*found_scope = scope->child(path);
      if (found_scope == 0) {
	    found_scope = scope->find_import(this, path.peek_name());
	    if (found_scope)
		  found_scope = found_scope->child(path);
      }
      return found_scope;
}

/*
 * This is a relative lookup of a scope by name. The starting point is
 * the scope parameter within which I start looking for the scope. If
 * I do not find the scope within the passed scope, start looking in
 * parent scopes until I find it, or I run out of parent scopes.
 */
NetScope* Design::find_scope(NetScope*scope, const hname_t&path,
                             NetScope::TYPE type) const
{
      assert(scope);

	// Record the compilation unit scope for use later.
      NetScope*unit_scope = scope->unit();

	// First search upwards through the hierarchy.
      while (scope) {
	    NetScope*found_scope = find_scope_(scope, path, type);
	    if (found_scope)
		  return found_scope;

	      // Avoid searching the unit scope twice.
	    if (scope == unit_scope)
		  unit_scope = 0;

	      // Special case - see IEEE 1800-2012 section 23.8.1.
	    if (unit_scope && is_design_unit(scope) && is_subroutine(type)) {
		  found_scope = find_scope_(unit_scope, path, type);
		  if (found_scope)
			  return found_scope;

		  unit_scope = 0;
	    }

	    scope = scope->parent();
      }

	// If we haven't already done so, search the compilation unit scope.
      if (unit_scope) {
	    NetScope*found_scope = find_scope_(unit_scope, path, type);
	    if (found_scope)
		  return found_scope;
      }

	// Last chance. Look for the name starting at the root.
      list<hname_t>path_list;
      path_list.push_back(path);
      return find_scope(path_list);
}

/*
 * This method runs through the scope, noticing the defparam
 * statements that were collected during the elaborate_scope pass and
 * applying them to the target parameters. The implementation actually
 * works by using a specialized method from the NetScope class that
 * does all the work for me.
 */
void Design::run_defparams()
{
      for (list<NetScope*>::const_iterator scope = root_scopes_.begin();
	   scope != root_scopes_.end(); ++ scope )
	    (*scope)->run_defparams(this);
}

void NetScope::run_defparams(Design*des)
{
      for (map<hname_t,NetScope*>::const_iterator cur = children_.begin()
		 ; cur != children_.end() ; ++ cur )
	    cur->second->run_defparams(des);

      while (! defparams.empty()) {
	    pair<pform_name_t,PExpr*> pp = defparams.front();
	    defparams.pop_front();

	    pform_name_t path = pp.first;
	    PExpr*val = pp.second;

	    perm_string perm_name = peek_tail_name(path);
	    path.pop_back();

	    list<hname_t> eval_path = eval_scope_path(des, this, path);

	      /* If there is no path on the name, then the targ_scope
		 is the current scope. */
	    NetScope*targ_scope = des->find_scope(this, eval_path);
	    if (targ_scope == 0) {

		    // Push the defparam onto a list for retry
		    // later. It is possible for the scope lookup to
		    // fail if the scope being defparam'd into is
		    // generated by an index array for generate.
		  eval_path.push_back(hname_t(perm_name));
		  defparams_later.push_back(make_pair(eval_path,val));
		  continue;
	    }

	    bool flag = targ_scope->replace_parameter(perm_name, val, this);
	    if (! flag) {
		  cerr << val->get_fileline() << ": warning: parameter "
		       << perm_name << " not found in "
		       << scope_path(targ_scope) << "." << endl;
	    }

      }

	// If some of the defparams didn't find a scope in the name,
	// then try again later. It may be that the target scope is
	// created later by generate scheme or instance array.
      if (! defparams_later.empty())
	    des->defparams_later.insert(this);
}

void NetScope::run_defparams_later(Design*des)
{
      set<NetScope*> target_scopes;
      list<pair<list<hname_t>,PExpr*> > defparams_even_later;

      while (! defparams_later.empty()) {
	    pair<list<hname_t>,PExpr*> cur = defparams_later.front();
	    defparams_later.pop_front();

	    list<hname_t>eval_path = cur.first;
	    perm_string name = eval_path.back().peek_name();
	    eval_path.pop_back();

	    PExpr*val = cur.second;

	    NetScope*targ_scope = des->find_scope(this, eval_path);
	    if (targ_scope == 0) {
		    // If a scope in the target path is not found,
		    // then push this defparam for handling even
		    // later. Maybe a later generate scheme or
		    // instance array will create the scope.
		  defparams_even_later.push_back(cur);
		  continue;
	    }

	    bool flag = targ_scope->replace_parameter(name, val, this);
	    if (! flag) {
		  cerr << val->get_fileline() << ": warning: parameter "
		       << name << " not found in "
		       << scope_path(targ_scope) << "." << endl;
	    }

	      // We'll need to re-evaluate parameters in this scope
	    target_scopes.insert(targ_scope);
      }

	// The scopes that this defparam set touched will be
	// re-evaluated later it a top_defparams work item. So do not
	// do the evaluation now.

	// If there are some scopes that still have missing scopes,
	// then save them back into the defparams_later list for a
	// later pass.
      defparams_later = defparams_even_later;
      if (! defparams_later.empty())
	    des->defparams_later.insert(this);
}

void Design::evaluate_parameters()
{
      for (map<perm_string,NetScope*>::const_iterator cur = packages_.begin()
		 ; cur != packages_.end() ; ++ cur) {
	    cur->second->evaluate_parameters(this);
      }

      for (list<NetScope*>::const_iterator scope = root_scopes_.begin()
		 ; scope != root_scopes_.end() ; ++ scope ) {
	    (*scope)->evaluate_parameters(this);
      }
}

void NetScope::evaluate_parameter_logic_(Design*des, param_ref_t cur)
{
      long msb = 0;
      long lsb = 0;
      bool range_flag = false;

	/* Evaluate the msb expression, if it is present. */
      PExpr*msb_expr = (*cur).second.msb_expr;
      if (msb_expr) {
            (*cur).second.msb = elab_and_eval(des, this, msb_expr, -1, true);
	    if (! eval_as_long(msb, (*cur).second.msb)) {
		  cerr << (*cur).second.val->get_fileline()
		       << ": error: Unable to evaluate msb expression "
		       << "for parameter " << (*cur).first << ": "
		       << *(*cur).second.msb << endl;
		  des->errors += 1;
		  return;
	    }

	    range_flag = true;
      }

	/* Evaluate the lsb expression, if it is present. */
      PExpr*lsb_expr = (*cur).second.lsb_expr;
      if (lsb_expr) {
            (*cur).second.lsb = elab_and_eval(des, this, lsb_expr, -1, true);
	    if (! eval_as_long(lsb, (*cur).second.lsb)) {
		  cerr << (*cur).second.val->get_fileline()
		       << ": error: Unable to evaluate lsb expression "
		       << "for parameter " << (*cur).first << ": "
		       << *(*cur).second.lsb << endl;
		  des->errors += 1;
		  return;
	    }

	    range_flag = true;
      }

	/* Evaluate the parameter expression. */
      PExpr*val_expr = (*cur).second.val_expr;
      NetScope*val_scope = (*cur).second.val_scope;

      int lv_width = -2;
      if (range_flag)
	    lv_width = (msb >= lsb) ? 1 + msb - lsb : 1 + lsb - msb;

      NetExpr*expr = elab_and_eval(des, val_scope, val_expr, lv_width, true,
                                   (*cur).second.is_annotatable,
                                   (*cur).second.type);
      if (! expr)
            return;

      switch (expr->expr_type()) {
	  case IVL_VT_REAL:
	    if (! dynamic_cast<const NetECReal*>(expr)) {
		  cerr << expr->get_fileline()
		       << ": error: Unable to evaluate real parameter "
		       << (*cur).first << " value: " << *expr << endl;
		  des->errors += 1;
		  return;
	    }
	    break;

	  case IVL_VT_LOGIC:
	  case IVL_VT_BOOL:
	    if (! dynamic_cast<const NetEConst*>(expr)) {
		  cerr << expr->get_fileline()
		       << ": error: Unable to evaluate parameter "
		       << (*cur).first << " value: " << *expr << endl;
		  des->errors += 1;
		  return;
	    }

	      // If the parameter has type or range information, then
	      // make sure the type is set right. Note that if the
	      // parameter doesn't have an explicit type or range,
	      // then it will get the signedness from the expression itself.
	    if (cur->second.type != IVL_VT_NO_TYPE) {
		  expr->cast_signed(cur->second.signed_flag);
	    } else if (cur->second.signed_flag) {
		  expr->cast_signed(true);
	    }

	    if (!range_flag && !expr->has_width()) {
		  expr = pad_to_width(expr, integer_width, *expr);
	    }
	    break;

	  default:
	    cerr << expr->get_fileline()
		 << ": internal error: "
		 << "Unhandled expression type?" << endl;
	    des->errors += 1;
	    return;
      }

      cur->second.val = expr;

	// If there are no value ranges to test the value against,
	// then we are done.
      if ((*cur).second.range == 0) {
	    return;
      }

      NetEConst*val = dynamic_cast<NetEConst*>((*cur).second.val);
      ivl_assert(*(*cur).second.val, (*cur).second.val);
      ivl_assert(*(*cur).second.val, val);

      verinum value = val->value();

      bool from_flag = (*cur).second.range == 0? true : false;
      for (range_t*value_range = (*cur).second.range
		 ; value_range ; value_range = value_range->next) {

	      // If we already know that the value is
	      // within a "from" range, then do not test
	      // any more of the from ranges.
	    if (from_flag && value_range->exclude_flag==false)
		  continue;

	    if (value_range->low_expr) {
		  NetEConst*tmp = dynamic_cast<NetEConst*>(value_range->low_expr);
		  ivl_assert(*value_range->low_expr, tmp);
		  if (value_range->low_open_flag && value <= tmp->value())
			continue;
		  else if (value < tmp->value())
			continue;
	    }

	    if (value_range->high_expr) {
		  NetEConst*tmp = dynamic_cast<NetEConst*>(value_range->high_expr);
		  ivl_assert(*value_range->high_expr, tmp);
		  if (value_range->high_open_flag && value >= tmp->value())
			continue;
		  else if (value > tmp->value())
			continue;
	    }

	      // Within the range. If this is a "from"
	      // range, then set the from_flag and continue.
	    if (value_range->exclude_flag == false) {
		  from_flag = true;
		  continue;
	    }

	      // OH NO! In an excluded range. signal an error.
	    from_flag = false;
	    break;
      }

	// If we found no from range that contains the
	// value, then report an error.
      if (! from_flag) {
	    cerr << val->get_fileline() << ": error: "
		 << "Parameter value " << value
		 << " is out of range for parameter " << (*cur).first
		 << "." << endl;
	    des->errors += 1;
      }
}

void NetScope::evaluate_parameter_real_(Design*des, param_ref_t cur)
{
      PExpr*val_expr = (*cur).second.val_expr;
      NetScope*val_scope = (*cur).second.val_scope;

      NetExpr*expr = elab_and_eval(des, val_scope, val_expr, -1, true,
                                   (*cur).second.is_annotatable,
                                   (*cur).second.type);
      if (! expr)
            return;

      NetECReal*res = 0;

      switch (expr->expr_type()) {
	  case IVL_VT_REAL:
	    if (NetECReal*tmp = dynamic_cast<NetECReal*>(expr)) {
		  res = tmp;
	    } else {
		  cerr << expr->get_fileline()
		       << ": error: "
		       << "Unable to evaluate real parameter "
		       << (*cur).first << " value: " << *expr << endl;
		  des->errors += 1;
		  return;
	    }
	    break;

	  default:
	    cerr << expr->get_fileline()
		 << ": internal error: "
		 << "Failed to cast expression?" << endl;
	    des->errors += 1;
	    return;
	    break;
      }

      (*cur).second.val = res;
      double value = res->value().as_double();

      bool from_flag = (*cur).second.range == 0? true : false;
      for (range_t*value_range = (*cur).second.range
		 ; value_range ; value_range = value_range->next) {

	    if (from_flag && value_range->exclude_flag==false)
		  continue;

	    if (value_range->low_expr) {
		  double tmp;
		  bool flag = eval_as_double(tmp, value_range->low_expr);
		  ivl_assert(*value_range->low_expr, flag);
		  if (value_range->low_open_flag && value <= tmp)
			continue;
		  else if (value < tmp)
			continue;
	    }

	    if (value_range->high_expr) {
		  double tmp;
		  bool flag = eval_as_double(tmp, value_range->high_expr);
		  ivl_assert(*value_range->high_expr, flag);
		  if (value_range->high_open_flag && value >= tmp)
			continue;
		  else if (value > tmp)
			continue;
	    }

	    if (value_range->exclude_flag == false) {
		  from_flag = true;
		  continue;
	    }

	      // All the above tests failed, so we must have tripped
	      // an exclude rule.
	    from_flag = false;
	    break;
      }

      if (! from_flag) {
	    cerr << res->get_fileline() << ": error: "
		 << "Parameter value " << value
		 << " is out of range for real parameter " << (*cur).first
		 << "." << endl;
	    des->errors += 1;
      }
}

void NetScope::evaluate_parameter_(Design*des, param_ref_t cur)
{
	// If the parameter has already been evaluated, quietly return.
      if (cur->second.val_expr == 0)
            return;

      if (cur->second.solving) {
            cerr << cur->second.get_fileline() << ": error: "
	         << "Recursive parameter reference found involving "
                 << cur->first << "." << endl;
	    des->errors += 1;
      } else {
            cur->second.solving = true;
            switch (cur->second.type) {
                case IVL_VT_NO_TYPE:
                case IVL_VT_BOOL:
                case IVL_VT_LOGIC:
                  evaluate_parameter_logic_(des, cur);
                  break;

                case IVL_VT_REAL:
                  evaluate_parameter_real_(des, cur);
                  break;

                default:
                  cerr << cur->second.get_fileline() << ": internal error: "
                       << "Unexpected expression type " << cur->second.type
                       << "." << endl;
                  cerr << cur->second.get_fileline() << ":               : "
                       << "Parameter name: " << cur->first << endl;
                  cerr << cur->second.get_fileline() << ":               : "
                       << "Expression is: " << *cur->second.val_expr << endl;
                  ivl_assert(cur->second, 0);
                  break;
            }
            cur->second.solving = false;
      }

        // If we have failed to evaluate the expression, create a dummy
        // value. This prevents spurious error messages being output.
      if (cur->second.val == 0) {
            verinum val(verinum::Vx);
            cur->second.val = new NetEConst(val);
      }

        // Flag that the expression has been evaluated.
      cur->second.val_expr = 0;
}

void NetScope::evaluate_parameters(Design*des)
{
      for (map<hname_t,NetScope*>::const_iterator cur = children_.begin()
		 ; cur != children_.end() ; ++ cur )
	    cur->second->evaluate_parameters(des);

      if (debug_scopes)
	    cerr << "debug: "
		 << "Evaluating parameters in " << scope_path(this) << endl;

      for (param_ref_t cur = parameters.begin()
		 ; cur != parameters.end() ;  ++ cur) {

            evaluate_parameter_(des, cur);
      }
}

void Design::residual_defparams()
{
      for (list<NetScope*>::const_iterator scope = root_scopes_.begin();
	   scope != root_scopes_.end(); ++ scope )
	    (*scope)->residual_defparams(this);
}

void NetScope::residual_defparams(Design*des)
{
	// Clean out the list of defparams that never managed to match
	// a scope. Print a warning for each.
      while (! defparams_later.empty()) {
	    pair<list<hname_t>,PExpr*> cur = defparams_later.front();
	    defparams_later.pop_front();

	    cerr << cur.second->get_fileline() << ": warning: "
		 << "Scope of " << cur.first << " not found." << endl;
      }

      for (map<hname_t,NetScope*>::const_iterator cur = children_.begin()
		 ; cur != children_.end() ; ++ cur )
	    cur->second->residual_defparams(des);
}

const char* Design::get_flag(const string&key) const
{
      map<string,const char*>::const_iterator tmp = flags_.find(key);
      if (tmp == flags_.end())
	    return "";
      else
	    return (*tmp).second;
}

/*
 * This method looks for a signal (reg, wire, whatever) starting at
 * the specified scope. If the name is hierarchical, it is split into
 * scope and name and the scope used to find the proper starting point
 * for the real search.
 *
 * It is the job of this function to properly implement Verilog scope
 * rules as signals are concerned.
 */
NetNet* Design::find_signal(NetScope*scope, pform_name_t path)
{
      assert(scope);

      perm_string key = peek_tail_name(path);
      path.pop_back();
      if (! path.empty()) {
	    list<hname_t> eval_path = eval_scope_path(this, scope, path);
	    scope = find_scope(scope, eval_path);
      }

      while (scope) {
	    if (NetNet*net = scope->find_signal(key))
		  return net;

	    if (NetScope*import_scope = scope->find_import(this, key)) {
		  scope = import_scope;
		  continue;
	    }

	    if (scope->type() == NetScope::MODULE)
		  break;

	    scope = scope->parent();
      }

      return 0;
}

NetFuncDef* Design::find_function(NetScope*scope, const pform_name_t&name)
{
      assert(scope);

      std::list<hname_t> eval_path = eval_scope_path(this, scope, name);
      NetScope*func = find_scope(scope, eval_path, NetScope::FUNC);
      if (func && (func->type() == NetScope::FUNC)) {
              // If a function is used in a parameter definition or in
              // a signal declaration, it is possible to get here before
              // the function's signals have been elaborated. If this is
              // the case, elaborate them now.
            if (func->elab_stage() < 2) {
		  func->need_const_func(true);
                  const PFunction*pfunc = func->func_pform();
                  assert(pfunc);
                  pfunc->elaborate_sig(this, func);
            }
	    return func->func_def();
      }
      return 0;
}

NetScope* Design::find_task(NetScope*scope, const pform_name_t&name)
{
      std::list<hname_t> eval_path = eval_scope_path(this, scope, name);
      NetScope*task = find_scope(scope, eval_path, NetScope::TASK);
      if (task && (task->type() == NetScope::TASK))
	    return task;

      return 0;
}

void Design::add_node(NetNode*net)
{
      assert(net->design_ == 0);
      if (nodes_ == 0) {
	    net->node_next_ = net;
	    net->node_prev_ = net;
      } else {
	    net->node_next_ = nodes_->node_next_;
	    net->node_prev_ = nodes_;
	    net->node_next_->node_prev_ = net;
	    net->node_prev_->node_next_ = net;
      }
      nodes_ = net;
      net->design_ = this;
}

void Design::del_node(NetNode*net)
{
      assert(net != 0);
      assert(net->design_ == this);

	/* Interact with the Design::functor method by manipulating the
	   cur and nxt pointers that it is using. */
      if (net == nodes_functor_nxt_)
	    nodes_functor_nxt_ = nodes_functor_nxt_->node_next_;
      if (net == nodes_functor_nxt_)
	    nodes_functor_nxt_ = 0;

      if (net == nodes_functor_cur_)
	    nodes_functor_cur_ = 0;

	/* Now perform the actual delete. */
      if (nodes_ == net)
	    nodes_ = net->node_prev_;

      if (nodes_ == net) {
	    nodes_ = 0;
      } else {
	    net->node_next_->node_prev_ = net->node_prev_;
	    net->node_prev_->node_next_ = net->node_next_;
      }

      net->design_ = 0;
}

void Design::add_branch(NetBranch*bra)
{
      bra->next_ = branches_;
      branches_ = bra;
}

void Design::add_process(NetProcTop*pro)
{
      pro->next_ = procs_;
      procs_ = pro;
}

void Design::add_process(NetAnalogTop*pro)
{
      pro->next_ = aprocs_;
      aprocs_ = pro;
}
void Design::delete_process(NetProcTop*top)
{
      assert(top);
      if (procs_ == top) {
	    procs_ = top->next_;

      } else {
	    NetProcTop*cur = procs_;
	    while (cur->next_ != top) {
		  assert(cur->next_);
		  cur = cur->next_;
	    }

	    cur->next_ = top->next_;
      }

      if (procs_idx_ == top)
	    procs_idx_ = top->next_;

      delete top;
}

void Design::join_islands(void)
{
      if (nodes_ == 0)
	    return;

      NetNode*cur = nodes_->node_next_;
      do {
	    join_island(cur);
	    cur = cur->node_next_;
      } while (cur != nodes_->node_next_);
}

//customed design for eda compitition by song changjun

		/*
		先不考虑单文件多module

		之后判断np->statement_的类型，本质上是树的遍历
		statement_(节点node)有以下类型：
			leaf:    
				NetAssignBase	---NetAssign     	阻塞赋值
                 			 	---NetAssignNB   	非阻塞赋值
                 				---NetCAssign    	连续赋值（存疑）
                 				---NetDeassign   	解除赋值
    		branch:	
				NetBlock                      		begin-end块
    			NetCase                       		case语句
    			NetCondit                     		if-else
				NetForLoop                          for 循环
				XXXXXXXXXX                          for循环嵌套
    			NetContribution               		不知道是啥意思
    			NetDisable                    		不知道啥意思
   	 			NetDoWhile                    		do-while
				NetUTask							task
				XXXXXXXXXX							function
				

		将np看做根(root)，NetAssignBase为需要遍历的叶节点(leaf)
		需要对分支节点(branch)的子女节点(child)特性逐一讨论

		NetBlock类
			const NetProc*proc_first() const;
      		const NetProc*proc_next(const NetProc*cur) const;

		NetCase类
			inline unsigned nitems() const { return items_.size(); }
			inline const NetExpr*expr(unsigned idx) const { return items_[idx].guard;}
      		inline const NetProc*stat(unsigned idx) const { return items_[idx].statement; }

		NetCondit类
			NetProc* if_clause();
      		NetProc* else_clause();

		NetDoWhile类
			NetExpr* cond_;
      		NetProc* proc_;

		用深度优先，递归地遍历
		PreOder()

		*/


/*
冲突的判定
同一模块不同always的同一信号，为冲突的
需要一个数据结构，维护赋值语句左值和行数，方便打印
*/

/*
函数名：tab			
参数：int d		打印次数
返回值：void
作用：打印\t
最新修改时间：2020/9/29
作者：宋长骏
*/

void tab(int depth)
{
	for(int i=0;i<depth;i++)
	{
		printf("\t");
	}
}
/*
函数名：recurseTraversal	递归遍历
参数：NetProc* tn   	Process类，可动态类型转换到赋值，条件，case等等类型   int d 深度，方便打印调试
返回值：void
作用：递归调用，找出赋值语句左值
编写时间：2020/9/29
修改日志：
	10.1：考虑到赋值语句多比特，无select的情况
作者：宋长骏
*/
void Design::recurseTraversal(NetProc* tn , int d )    
    {

        if (tn)   
        {   int depth=d+1;
            char * type_name= new char [20];           //用来存type_info
			std::strcpy(type_name,typeid(*tn).name());
			
			if (!std::strcmp(type_name,"9NetAssign"))              //阻塞赋值
			{
				NetAssign* ass = dynamic_cast<NetAssign *>(tn);
				unsigned cnt = ass->l_val_count();
				assert(cnt>0);
				NetAssign_*lval = ass->l_val(0);
				NetNet* net =  lval->sig();
				unsigned wid=lval->lwidth();
				if(lval->get_base())                        //如果base非空,则存在select(如a[3:1])
				{
					//如果base是用常数表示的，则为NetEConst类型，是NetExpr子类
					//如果base中有信号，NetESignal。calculates the index of the word in thearray. It may only be nil if the expression refers to the whole array
					
					char * type_name0= new char [20];           //用来存type_info
					std::strcpy(type_name0,typeid(*(lval->get_base())).name());
					if (!std::strcmp(type_name0,"9NetEConst"))              //下标为常数
					{
						NetExpr* base_expr_tmp=const_cast<NetExpr*>(lval->get_base());
						NetEConst* base_expr=dynamic_cast<NetEConst *>(base_expr_tmp);
						unsigned len=base_expr->value().len();
						unsigned base_num=0;
						for (int i=0;i<len;i++)
						{	base_num+=((base_expr->value().get(i))<<i);
						}
						tab(depth);
						printf("var %s[%d:%d] in line %d is the l_val of Assign ! \n:" , (net->name().str()),base_num+wid-1,base_num,tn->get_lineno());
					}
					else if (!std::strcmp(type_name0,"10NetESignal"))              //下标含信号，先假定是单信号
					{
						NetExpr* base_expr_tmp=const_cast<NetExpr*>(lval->get_base());
						NetESignal* base_expr=dynamic_cast<NetESignal *>(base_expr_tmp);
						tab(depth);
						printf("var %s[%s] in line %d is the l_val of Assign ! \n:" , (net->name().str()),base_expr->sig()->name().str(),tn->get_lineno());
					}
					delete[] type_name0;

				}
				else{
					tab(depth);
					printf("var %s[%d:0] in line %d is the l_val of Assign !\n:" , (net->name().str()),wid-1,tn->get_lineno());
				}
				
			
			}
			else if (!std::strcmp(type_name,"11NetAssignNB"))		//非阻塞赋值
			{
				NetAssignNB* ass = dynamic_cast<NetAssignNB *>(tn);
				unsigned cnt = ass->l_val_count();
				assert(cnt>0);
				NetAssign_* lval = ass->l_val(0);
				NetNet* net =  lval->sig();
				unsigned wid=lval->lwidth();
				if(lval->get_base())
				{
				//NetEConst是NetExpr子类
				NetExpr* base_expr_tmp=const_cast<NetExpr*>(lval->get_base());
				NetEConst* base_expr=dynamic_cast<NetEConst *>(base_expr_tmp);
				unsigned len=base_expr->value().len();
				unsigned base_num=0;
				for (int i=0;i<len;i++)
				{	base_num+=((base_expr->value().get(i))<<i);
					
				}
				tab(depth);
				printf("var %s[%d:%d] in line %d is the l_val of AssignNB ! \n:" , (net->name().str()),base_num+wid-1,base_num,tn->get_lineno());
				}
				else{
				tab(depth);
				printf("var %s[%d:0] in line %d is the l_val of Assign !\n:" , (net->name().str()),wid-1,tn->get_lineno());
				}
				
			}
			else if (!std::strcmp(type_name,"9NetCondit"))		    //条件语句 if else中写同一bit，不算冲突
			{
				NetCondit* con = dynamic_cast<NetCondit *>(tn);
				tab(depth);
				printf("CONDITION in line %d !\n:" ,tn->get_lineno());
				//if句递归
				recurseTraversal(con->if_clause(),depth);  
				//else句递归
				recurseTraversal(con->else_clause(),depth);
				
			}
			else if (!std::strcmp(type_name,"7NetCase"))		    //case语句
			{
				NetCase* cas = dynamic_cast<NetCase *>(tn);
				tab(depth);
				printf("CASE in line %d !\n:" ,tn->get_lineno());
				
				size_t len = cas->nitems();     //分支个数
    			for (size_t i =0; i < len; i ++) {

        			recurseTraversal(const_cast<NetProc*>(cas->stat(i)),depth);         //const传不了，得先转换 
    			}
			}
			else if (!std::strcmp(type_name,"8NetBlock"))		    //begin-end块
			{
				NetBlock* blo = dynamic_cast<NetBlock *>(tn);
				tab(depth);
				printf("BLOCK in line %d !\n:" ,tn->get_lineno());
				for (const NetProc* cur = blo->proc_first();cur; cur=blo->proc_next(cur)) 
				{
					recurseTraversal(const_cast<NetProc*>(cur),depth);
				}                   
			}
			else if (!std::strcmp(type_name,"10NetForLoop"))		    //for循环
			{
				NetForLoop* fo = dynamic_cast<NetForLoop *>(tn);
				tab(depth);
				printf("in line %d is FOR_LOOP !\n:" ,tn->get_lineno());
				//for 涉及到循环变量index赋初值，更新时的赋值操作，循环体的赋值操作，i从多少到多少也很重要
				//NetNet*index_; 循环变量
      			//NetProc*statement_;
      			//NetProc*step_statement_;	  //找出变化方式

				//NetExpr*init_expr_;        //找出初始 用常数 NetEConst
      			//NetExpr*condition_;		//找出终止  比较 NetEBComp
				//打印循环变量的信息（名字，比特）
				
				NetEConst* base_expr=dynamic_cast<NetEConst *>(fo->init_expr_);
				unsigned len=base_expr->value().len();
				unsigned base_num=0;     //循环初始值
				for (int i=0;i<len;i++)
				{	base_num+=((base_expr->value().get(i))<<i);
					
				}
				NetEBComp* cmp_expr=dynamic_cast<NetEBComp *>(fo->condition_);	


				tab(depth);
				printf("the index of FOR_LOOP is : %s[%d:%d] , initial value: %d  !\n:" ,fo->index_->name().str(),fo->index_->slice_dims_[0].get_msb(),fo->index_->slice_dims_[0].get_lsb(),base_num);
				//NetForLoop* fo = dynamic_cast<NetForLoop *>(tn);
				//NetForLoop* fo = dynamic_cast<NetForLoop *>(tn);
				//更新操作的内容
				recurseTraversal(fo->step_statement_,depth);

				//循环体的内容
				recurseTraversal(fo->statement_,depth);


			}
			else if (!std::strcmp(type_name,"8NetUTask"))		    //task不处理了，会对应一个block，和对应的input,output赋值，我还不清楚机制
			{
				NetUTask* ta = dynamic_cast<NetUTask *>(tn);
				tab(depth);
				printf("in line %d is Task !\n:" ,tn->get_lineno());
				//printf("in line %d is Task , name is : %s !\n:" ,tn->get_lineno(),ta->name());

				//不太会
				//recurseTraversal(ta->next_,depth);          //我改了友元类
			}
			else           //未考虑到的情况
			{
				tab(depth);
				printf("in line %d is %s !\n:" ,tn->get_lineno(),type_name);
			}
			
			delete[] type_name;
        }       
	  	
    } 


void Design::examples()
{
	std::map<NetNet*,int> net_map;														

	for (NetProcTop* cur = procs_; cur; cur = cur->next_){					//遍历always/initial块
		printf("ALWAYS of module : %s , in line : %d !\n:" ,cur->scope_->fullname().peek_name().str(), cur->get_lineno());
		char * type_name= new char [20];           //用来存type_info
		std::strcpy(type_name,typeid(*(cur->statement())).name());
		//printf("type_name of cur->statement: %s \n",type_name);
		delete[] type_name;
		//ret = strcmp(str1, str2);
		NetEvWait* np = dynamic_cast<NetEvWait *>(cur->statement());         //通常是NetEvWait
		if (!np) continue;        //如果不是，就跳过了
		recurseTraversal(np->statement(),0);
	

	
	
	/*
		std::strcpy(type_name,typeid(*(np->statement())).name());       
		printf("type_name of cur->statement: %s \n",type_name);

		NetCondit* ass = dynamic_cast<NetCondit*>(np->statement());
		
		if (!ass) continue;
	*/
	/*
		unsigned cnt = ass->l_val_count();
		assert(cnt>0);
		NetAssign_* lval = ass->l_val(0);
		NetNet* net =  lval->sig();
		printf("var %s !\n:" , (net->name().str()));
		if (net_map.find(net) == net_map.end())
			net_map[net]=1;
		else
		{
			printf("var %s has multi_driver in line %d !\n:" , (net->name().str()),net->get_lineno());
			
		}
	*/
		
	}
}
/*
class sxw_Bit{
      private: 
      sxw_Bit();
      ~sxw_Bit();
      public:
      list<unsigned> row_list_;     //出现的行数 
      perm_string name_;            //全名，比如a_0,b_1这种

};
*/
sxw_Bit::sxw_Bit()
{

	
}
#pragma GCC pop_options