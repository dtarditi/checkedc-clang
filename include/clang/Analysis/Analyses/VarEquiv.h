//=------ VarEquiv.cpp - Analysis of equality of variables -----*- C++ --**-==/
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//  Conservatively determine for each program point in a function which 
//  variables must be equal to each other, constants, or address-expressions 
//  whose values do not vary during during the lifetime of the function.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_ANALYSIS_ANALYSES_VAREQUIV_H
#define LLVM_CLANG_ANALYSIS_ANALYSES_VAREQUIV_H

#include "clang/Analysis/CFG.h"
#include "clang/AST/Decl.h"
#include "clang/AST/Stmt.h"
#include "clang/AST/CanonBounds.h"

namespace clang {

class SourceManager;

namespace PartitionRefinement {
  typedef int Element;
  class ElementMap;
  struct ListNode;
  class Set;
  class SetManager;

  // Represent a partitioning of a set of integers into
  // equivalence classes.  Provide operations for refining
  // the partition (dividing existing classes into smaller
  // classes).
  class Partition {
  public:
    Partition();
    ~Partition();
    /// \brief Add Elem to the equivalence class for Member.  Elem must
    /// currently be a member of a singleton equivalence class and
    /// cannot be a member of a non-trivial equivalence class.
    void add(Element Member, Element Elem);
    /// \brief Update this partition to have the same equivalence
    /// classes as P.
    void assign(Partition &P);
    /// \brief  Place all elements in singleton equivalence classes.
    void clear();
    /// \brief Returns true if Elem is a member of a singleton
    /// equivalence class.
    bool isSingleton(Element Elem) const;
    /// \brief Refine the partition so that Elem is in a singleton
    /// equivalence class.
    void makeSingleton(Element Elem);
    /// \brief Return a representative element of the equivalence class
    /// containing Elem.
    Element getRepresentative(Element Elem) const;
    /// \brief Refine the existing partition by each of the equivalence
    /// classes in R.  For each equivalence class C in 'this' partition that
    /// contains an element of R, divide it into two equivalence classes:
    /// intersection(C, R) and C - R.
    void refine(const Partition *R);
    void dump(raw_ostream &OS, Element Elem) const;
    void dump(raw_ostream &OS) const;

  private:
    ListNode *add(Set *S, Element Elem);
    void remove_if_trivial(Set *S);
    void refine(const Set *S);
    void dump(raw_ostream &OS, Set *S) const;

    ElementMap *NodeMap;
    SetManager *Sets;
    std::vector<Set *> Scratch;
  };
}

// Analysis for computing variables holding equivalent values.  The analysis
// should be used in the following way:
// - Construct an instance of the analysis.
// - Call analyze to compute per-basic block information.
// - For each basic block,
// -- Call setCurrentBlock.
// -- Iterate through the statements in the basic block, calling addEffects
//    to add the effects of the current statement on variable equivalence.
// -- Use  getRepresentative to get the representative variable for a
//    variable at the current program point (given a set of variables holding
//    equivalent variables, one is designated as the representative variable.
//    Two variables hold equivalent values if they have the same
//    representative variable.)

class InterestingEquivVars;

class VarEquiv : EqualityRelation {
public:
  VarEquiv(CFG *Cfg, Stmt *Body);
  ~VarEquiv();

  void analyze();
  void setCurrentBlock(CFGBlock block);
  void addEffects(Stmt *S);
  const VarDecl *getRepresentative(const VarDecl *D);

  /// \brief Print the equivalence information associated with
  /// each basic block to the output stream OS.
  void dumpAll(raw_ostream &OS, const SourceManager& M);
  /// \brief Print the equivalence information associated with
  /// the current program point the output stream OS
  void dumpCurrentStmt(raw_ostream &OS);

  /// \brief If E is a DeclRef to a local or parameter variable, return the
  /// variable.  Otherwise return null.
  static VarDecl *getVariable(Expr *E) {
    DeclRefExpr *DR = dyn_cast<DeclRefExpr>(E);
    if (DR != nullptr) {
      VarDecl *D = dyn_cast<VarDecl>(DR->getDecl());
      if (D && D->isLocalVarDeclOrParm())
        return D;
    }
    return nullptr;
  }

private:
  void analyzeExpr(Expr *Expr, PartitionRefinement::Partition *Equivalences);

  void analyzeBinaryOperator(BinaryOperator *BO,
                             PartitionRefinement::Partition *P);

  void analyzeConditionalOperator(ConditionalOperator *CO,
                                  PartitionRefinement::Partition *P);

  void analyzeBinaryConditionalOperator(BinaryConditionalOperator *BCO,
                                        PartitionRefinement::Partition *P);

  Stmt *Body;
  CFG *Cfg;
  PartitionRefinement::Partition *EquivalentVars;
  InterestingEquivVars *InterestingVars;
};
}
#endif
