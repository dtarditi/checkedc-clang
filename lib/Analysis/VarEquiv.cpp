//=------ VarEquiv.cpp - Analysis of equality of variables -----*- C++ --**-==//
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
//  whose values do not vary during the lifetime of the function.
//
//===----------------------------------------------------------------------===//

#include "clang/Analysis/Analyses/VarEquiv.h"
#include "clang/AST/StmtVisitor.h"

using namespace clang;

// Represent equivalence classes of a set of integers, providing efficient
// operations for partition refinement of the equivalence classes.
//
// Partition refinement works as follows: suppose we have a partitioning of a
// set of/ integers into sets S1, ... SN. Given a set R (the refinement set),
// divide each set SI that  contains a member of R into two new sets:
// - SI intersected with R
// - SI minus R
//
// This "refines" the partition into equal or smaller sets.  This operation
// can be used to intersect multiple sets of equivalence lasses efficiently.
//
// Partition refinement can be implemented so that the intersection can
// be done in time O(R) and space O(maximum integer in the set) by using
// a clever representation.  The representation is described at
// https://en.wikipedia.org/wiki/Partition_refinement.
//
// We expect equivalence classes to be sparse (consist mostly of singleton
// equivalence classes), so we implement partition refinment in a more
// space-efficient fashion. Our implementation uses time O(R * log(N)),
// where N is the number of integers not in singleton/ equivalence classes,
// and uses space O(N) to represent a partition.
//
// The original algorithm requires a mapping operation from individual
// integers that is constant time.  This is done using a dense array.
// We use a tree instead.   We did not use a hash table because we
// expect on average for there to be only a few integers in non-singleton
// equivalence classes, and a hash table would be less efficient for that.

namespace clang {
namespace PartitionRefinement {
  typedef int Element;

// We represent each equivalence class as an unordered doubly-linked list so 
// that we can easily add/remove elements from the set.
class Set;

struct ListNode {
  ListNode(Element E, Set *S) : Elem(E), ContainingSet(S), Prev(nullptr), Next(nullptr) {
  }

  Element Elem;
  Set *ContainingSet;

  ListNode *Prev;
  ListNode *Next;
};

class Set {
public:
  struct ListNode *Head;
  Set *Intersected; // scratch pointer used during refinement.
  int InternalId;   // Internal id of set use by SetManager. May change
                    // as sets are removed/added.

  Set() : Head(nullptr), Intersected(nullptr), InternalId(Sentinel) {}

  bool isEmpty() {
    return Head == nullptr;
  }

  bool isSingleton() const {
    return Head != nullptr && Head->Prev == nullptr && Head->Next == nullptr;
  }

  static const int Sentinel = -1;
};

// SetManager tracks a list of sets.
class SetManager {
private:
  std::vector<Set *> Sets;

public:
  // Add S to the list of sets. Return the position where S was
  // added.  We don't set the id because sometimes a set needs
  // to be tracked in two lists.
  void add(Set *S) {
    assert(S->InternalId == Set::Sentinel);
    S->InternalId = Sets.size();
    Sets.push_back(S);
  }

  void assignTo(std::vector<Set *> &Target) const {
    Target.assign(Sets.begin(), Sets.end());
  }

 
  Set *get(unsigned i) const {
    if (i < Sets.size())
      return Sets[i];
    else
      return nullptr;
  }

  // Remove a set from the list of set by swapping the 
  // set at the end of the list with this set.  Updates
  // the Id for the swapped set.
  void remove(Set *S) {
    int Id = S->InternalId;
    int size = Sets.size();
    assert(Id >= 0 && Id < size);
    if (Id != size - 1) {
      Set *SwapTarget = Sets[size - 1];
      Sets[Id] = SwapTarget;
      SwapTarget->InternalId = Id;
    }
    Sets.pop_back();
  }

  void clear() {
    Sets.clear();
  }

  int size() const {
    return Sets.size();
  }
};

// Map an equivalence set element to its list node.
class ElementMap {
private:
    std::map<Element, ListNode *> Tree;

public:
  ElementMap() {
  }
      
  ListNode *get(Element Elem) const {
    auto Lookup = Tree.find(Elem);
    if (Lookup == Tree.end())
      return nullptr;
    else
      return Lookup->second;
  }

  void remove(Element Elem) {
    auto Lookup = Tree.find(Elem);
    if (Lookup == Tree.end())
      return;
    Tree.erase(Lookup);
  }

  void set(Element Elem,ListNode *Node) {
    Tree[Elem] = Node;
  }
};

// Unlink the node from its current set.
void unlinkNode(ListNode *Node) {
  if (Node->Prev) {
    Node->Prev->Next = Node->Next;
  }
  if (Node->Next) {
    Node->Next->Prev = Node->Prev;
  }
  if (Node == Node->ContainingSet->Head) {
    assert(Node->Prev == nullptr);
    Node->ContainingSet->Head = Node->Next;
  }
}

// Link the node to set S
void linkNode(Set *S, ListNode *Node) {
  Node->ContainingSet = S;
  ListNode *Head = S->Head;
  Node->Prev = nullptr;
  Node->Next = Head;
  S->Head = Node;
  if (Head != nullptr) {
    Head->Prev = Node;
  }
}

// Move a node from its current set to set S.
void moveNode(Set *S, ListNode *Node) {
  unlinkNode(Node);
  linkNode(S, Node);
}

void Partition::remove_if_trivial(Set *S) {
  if (S->isEmpty()) {
    Sets->remove(S);
    delete S;
  }
  else if (S->isSingleton()) {
    Sets->remove(S);
    NodeMap->remove(S->Head->Elem);
    delete S->Head;
    delete S;
  }
}

Partition::Partition() {
  Sets = new SetManager();
  NodeMap = new ElementMap();
}

Partition::~Partition() {
  delete Sets;
  delete NodeMap;
}

// Add Elem to the set S.  It is an error if Elem is already a member 
// of another set.
ListNode *Partition::add(Set *S, Element Elem) {
  assert((NodeMap->get(Elem) == nullptr || 
          NodeMap->get(Elem)->ContainingSet == S) &&
         "add operation makes this no longer a partition");
  ListNode *Node = new ListNode(Elem, S);
  NodeMap->set(Elem, Node);
  linkNode(S, Node);
  return Node;
}

// Add Elem to the set S for Member.  If Member does not have
// a set, create a new set to contain Elem and Member.
// It is an error if Elem is already a member of another set.
void Partition::add(Element Member, Element Elem) {
  if (Member == Elem)  // nothing to do - this is a singleton set.
    return;

  ListNode *Node = NodeMap->get(Member);
  if (Node == nullptr) {
    Set *S = new Set();
    Sets->add(S);
    Node = add(S , Member);
  }
  add(Node->ContainingSet, Elem);
}

// Make the current partition be the same partition as P.  Do this
// by making a semantic copy. We don't attempt to make a deep copy of the
// P because that would be quite complicated to do.
void Partition::assign(Partition &P) {
  clear();
  unsigned count = P.Sets->size();
  for (unsigned i = 0; i < count; i++) {
    Set *S = P.Sets->get(i);
    ListNode *Start = S->Head;
    ListNode *Current = S->Head->Next;
    while (Current != nullptr)
      add(Start->Elem, Current->Elem);
  }
}

void Partition::clear() {
  Partition Empty;
  refine(&Empty);
}

bool Partition::isSingleton(Element Elem) const {
  return NodeMap->get(Elem) == nullptr;
}

// Make Elem a singleton equivalance class and remove it
// from any other equivalence classes that is a member of.
void Partition::makeSingleton(Element Elem) {
  ListNode *Node = NodeMap->get(Elem);
  if (Node != nullptr) {
    Set *S = Node->ContainingSet;
    assert(S != nullptr);
    unlinkNode(Node);
    NodeMap->remove(Elem);
    delete Node;
    remove_if_trivial(S); // S may point to freed memory after this.
  }
}

  // For each equivalence class C with a member in S,
  // split C into two sets:
  // * C intersected with S
  // * C - S.
  //
  // S must be a set from a different PartitionEquivalence.
  // We create a new set for C intersectd with S and use
  // the original set for S to hold C - S.

void Partition::refine(const Set *S) {
  Scratch.clear();

  for (ListNode *Current = S->Head; Current != nullptr;
       Current = Current->Next) {
    Element CurrentElem = Current->Elem;
    ListNode *Target = NodeMap->get(CurrentElem);
    if (Target != nullptr) {
      Set *TargetSet = Target->ContainingSet;
      Set *Intersected = TargetSet->Intersected;
      if (Intersected == nullptr) {
        Intersected = new Set();
        Sets->add(Intersected);
        TargetSet->Intersected = Intersected;
        Scratch.push_back(TargetSet);
      }
      moveNode(Intersected, Target);
    }
  }

  unsigned count = Scratch.size();
  for (unsigned i = 0; i < count; i++) {
    Set *Split = Scratch[i];
    Set *Intersected = Split->Intersected;
    Split->Intersected = nullptr;
    remove_if_trivial(Split);
    remove_if_trivial(Intersected);
  }
  Scratch.clear();
}

void Partition::refine(const Partition *R) { // TODO: mark R as const?
  assert(R != this);
  // First check that all elements of the current equivalence classes
  // are members of at least one equivalence class in R.  Elements
  // not in any equivalence class in R are in singleton equivalence
  // classes.  These are not represented in sets and need to be deleted.

  Sets->assignTo(Scratch);  // avoid modifying list of sets we are iterating over.
  unsigned count = Scratch.size();
  for (unsigned i = 0; i < count; i++) {
    Set *S = Scratch[i];
    ListNode *Current = S->Head;
    while (Current != nullptr) {
      // Save next pointer now in case we delete Current.
      ListNode *Next = Current->Next;
      if (R->isSingleton(Current->Elem)) {
        unlinkNode(Current);
        NodeMap->remove(Current->Elem);
        delete Current;
      }
      Current = Next;
    }
    remove_if_trivial(S); 
  }


  count = R->Sets->size();
  for (unsigned i = 0; i < count; i++)
    refine(R->Sets->get(i));
}

// Return a representative element from the equivalence set
// for Elem.
Element Partition::getRepresentative(Element Elem) const {
  ListNode *Node = NodeMap->get(Elem);
  if (Node == nullptr) {
    return Elem;
  } else {
    return Node->ContainingSet->Head->Elem;
  }
}

void Partition::dump(raw_ostream &OS, Set *S) const {
  ListNode *Current = S->Head;
  OS << "Set ";
  OS << "(Internal Id " << S->InternalId << ") ";
  OS << "{";
  bool first = true;
  while (Current != nullptr) {
    if (!first)
      OS << ", ";
    OS << Current->Elem;
    first = false;
    Current = Current->Next;
  }
  OS << "}";
}

// Dump the set that Elem is equivalent to.
void Partition::dump(raw_ostream &OS, Element Elem) const {
  ListNode *Node = NodeMap->get(Elem);
  OS << Elem;
  OS << ": ";
  if (Node == nullptr) {
    OS << "Itself";
    return;
  }
  dump(OS, Node->ContainingSet);
}

// Dump all the sets
void Partition::dump(raw_ostream &OS) const {
  unsigned Count = Sets->size();
  if (Count == 0)
    OS << "Equivalence classes are all trivial\n";
  else
    OS << "Non-trivial equivalence classes:\n";
      
  for (unsigned i = 0; i < Count; i++) {
    dump(OS, Sets->get(i));
    OS << "\n";
  }
}
} // namespace PartitionRefinement

namespace {
class AddressTakenAnalysis: public StmtVisitor<AddressTakenAnalysis> {
private:
  SmallVector<VarDecl *, 8> Vars;

public:
  AddressTakenAnalysis() {}

  void VisitUnaryAddrOf(const UnaryOperator *E) {
    Expr *Operand = E->getSubExpr();
    DeclRefExpr *DR = dyn_cast<DeclRefExpr>(Operand);
    if (DR) {
      VarDecl *VD = dyn_cast<VarDecl>(DR->getDecl());
      if (VD)
        Vars.push_back(VD);
      return;
    }
  }

  void analyze(Stmt *Body) {
    Vars.clear();
    Visit(Body);
    std::sort(Vars.begin(), Vars.end());
    std::unique(Vars.begin(), Vars.end());
  }

  bool isAddressTaken(VarDecl *VD) {
    return std::binary_search(Vars.begin(), Vars.end(), VD);
  }
};
}

class InterestingEquivVars : public StmtVisitor<InterestingEquivVars> {
private:
  int VarCount = 0;
  std::map<const VarDecl *,int> VarMap;
  std::vector<VarDecl *> ReverseMap;
  AddressTakenAnalysis AddressTaken;


public:
  InterestingEquivVars() {
  }

  void VisitBinaryOperator(const BinaryOperator *BO) {
    if (BO->getOpcode() == BinaryOperatorKind::BO_Assign) {
      VarDecl *LHSVar = getInterestingVariable(BO->getLHS());
      VarDecl *RHSVar = getInterestingVariable(BO->getRHS());
      if (LHSVar && RHSVar) {
        addVariable(LHSVar);
        addVariable(RHSVar);
      }
    }
  }

  void analyze(Stmt *Body) {
    AddressTaken.analyze(Body);
    VarCount = 0;
    VarMap.clear();
    Visit(Body);
  }

  int getNumber(const VarDecl *VD) {
    auto Lookup = VarMap.find(VD);
    if (Lookup == VarMap.end())
      return sentinel;
    else
      return Lookup->second;
  }

  VarDecl *getDecl(int i) {
    if (i >= 0 && (unsigned) i < ReverseMap.size())
      return ReverseMap[i];
    return nullptr;
  }

  static const int sentinel = -1;

private:
  VarDecl *getInterestingVariable(Expr *E) {
    VarDecl *D = VarEquiv::getVariable(E);
    if (D != nullptr && !AddressTaken.isAddressTaken(D))
      return D;
    else 
      return nullptr;
  }

  void addVariable(VarDecl *D) {
    auto Lookup = VarMap.find(D);

    if (Lookup == VarMap.end()) {
      int Position = ReverseMap.size();
      VarMap[D] = Position;
      ReverseMap.push_back(D);
    }
  }
};

VarEquiv::VarEquiv(CFG *Cfg, Stmt *Body) : Body(Body), Cfg(Cfg) {
  InterestingVars = new InterestingEquivVars();
  EquivalentVars = new PartitionRefinement::Partition();
}

VarEquiv::~VarEquiv() {
  delete InterestingVars;
  delete EquivalentVars;
}

void VarEquiv::analyze() {
   // Find the set of interesting variables and number them.
   InterestingVars->analyze(Body);
   // For now, do no intraprocedural analysis.  The set of
   // equivalent variables at the beginning of each basic block should
   // just be empty.
}

void VarEquiv::setCurrentBlock(CFGBlock block) {
  EquivalentVars->clear();
}

void VarEquiv::addEffects(Stmt *S) {
  Expr *E = dyn_cast<Expr>(S);
  if (E)
    analyzeExpr(E, EquivalentVars);
  else
    llvm_unreachable("expected an expression statement");
}

void VarEquiv::analyzeExpr(Expr *E, PartitionRefinement::Partition *P) {
  switch (E->getStmtClass()) {
#define ABSTRACT_STMT(Kind)
#define STMT(Kind, Base) case Expr::Kind##Class:
#define EXPR(Kind, Base)
#include "clang/AST/StmtNodes.inc"
    llvm_unreachable("cannot analyze a statement in analyzeExpr");
    case Expr::ConditionalOperatorClass: {
      ConditionalOperator *CO = dyn_cast<ConditionalOperator>(E);
      assert(CO && "dyn_cast failed");
      if (CO)
        analyzeConditionalOperator(CO, P);
      return;
    }
    case Expr::BinaryConditionalOperatorClass: {
      BinaryConditionalOperator *BCO =
        dyn_cast<BinaryConditionalOperator>(E);
      assert(BCO && "dyn_cast failed");
      if (BCO)
        analyzeBinaryConditionalOperator(BCO, P);
      return;
    }
    case Expr::BinaryOperatorClass: {
      BinaryOperator *BO = dyn_cast<BinaryOperator>(E);
      assert(BO && "dyn_cast failed");
      if (BO)
        analyzeBinaryOperator(BO, P);
      return;
    }
    default:
      break;
  }

  // Check children
  for (auto I = E->child_begin(); I != E->child_end(); ++I) {
    Expr *Child = dyn_cast<Expr>(*I);
    assert(Child);
    if (Child)
      analyzeExpr(Child, P);
  }
}

void VarEquiv::analyzeConditionalOperator(ConditionalOperator *CO,
                                          PartitionRefinement::Partition *P) {
   PartitionRefinement::Partition Tmp;
   // Compute the variable equivalences that are true after the conditional
   // expression is evaluated.
   VarEquiv::analyzeExpr(CO->getCond(), P);
   // Use those equivalences to compute the equivalences true after evaluating
   // the LHS and RHS.
   PartitionRefinement::Partition RHS;
   RHS.assign(*P);
   VarEquiv::analyzeExpr(CO->getLHS(), P);
   VarEquiv::analyzeExpr(CO->getRHS(), &RHS);
   // Intersect them to determine what is true after the expression.
   P->refine(&RHS);
}

void VarEquiv::analyzeBinaryConditionalOperator(BinaryConditionalOperator *BCO,
                                          PartitionRefinement::Partition *P) {
  // This is the GNU binary conditional operator e1 ? : e2, which is the same
  // as e1 ? e1 : e2, except that e1 is executed only once.
  VarEquiv::analyzeExpr(BCO->getCond(), P);
  PartitionRefinement::Partition RHS;
  // Compute the variables equivalences true after evaluating the RHS.
  RHS.assign(*P);
  // Intesect the variable equivalences.
  VarEquiv::analyzeExpr(BCO->getFalseExpr(), &RHS);
  P->refine(&RHS);
}

void VarEquiv::analyzeBinaryOperator(BinaryOperator *BO,
                                     PartitionRefinement::Partition *P) {
  if (BO->isLogicalOp()) {
    // Only the first expression is guaranteed to be executed.
    VarEquiv::analyzeExpr(BO->getLHS(), P);
    // However, the RHS may evaluated, invalidating some variable
    // equivalences.  Compute the equivalances true after the RHS.
    PartitionRefinement::Partition RHSPartition;
    RHSPartition.assign(*P);
    VarEquiv::analyzeExpr(BO->getRHS(), &RHSPartition);
    // Intersect them.
    P->refine(&RHSPartition);
    return;
  }

  VarEquiv::analyzeExpr(BO->getLHS(), P);
  VarEquiv::analyzeExpr(BO->getRHS(), P);

  if (!BO->isAssignmentOp())
    return;

  VarDecl *LHS = getVariable(BO->getLHS());
  if (LHS == nullptr)
    return;

  int LHSId = InterestingVars->getNumber(LHS);
  if (LHSId == InterestingEquivVars::sentinel)
    return;

  EquivalentVars->makeSingleton(LHSId);

  if (BO->getOpcode() != BinaryOperatorKind::BO_Assign)
    return;

  VarDecl *RHS = getVariable(BO->getRHS());
  if (RHS == nullptr)
    return;

  int RHSId = InterestingVars->getNumber(RHS);
  if (RHSId == InterestingEquivVars::sentinel)
    return;

  if (LHSId == RHSId) 
    return;

  EquivalentVars->add(RHSId, LHSId);
}

const VarDecl *VarEquiv::getRepresentative(const VarDecl *D) {
  int Id = InterestingVars->getNumber(D);
  if (Id != InterestingEquivVars::sentinel) {
    int ResultId = EquivalentVars->getRepresentative(Id);
    if (ResultId != Id)
      return InterestingVars->getDecl(ResultId);
  }
  return D;
}

void VarEquiv::dumpAll(raw_ostream &OS, const SourceManager& M) {
}

void VarEquiv::dumpCurrentStmt(raw_ostream &OS) {
   EquivalentVars->dump(OS);
}
} // namespace Clang